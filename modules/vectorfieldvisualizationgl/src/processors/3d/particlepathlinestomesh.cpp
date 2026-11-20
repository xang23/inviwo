/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2015-2026 Inviwo Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************************/

#include <modules/vectorfieldvisualizationgl/processors/3d/particlepathlinestomesh.h>
#include <modules/opengl/buffer/buffergl.h>

#include <inviwo/core/datastructures/buffer/bufferram.h>
#include <inviwo/core/datastructures/geometry/geometrytype.h>
#include <inviwo/core/datastructures/geometry/mesh.h>
#include <inviwo/core/util/exception.h>
#include <inviwo/core/util/staticstring.h>

#include <cstdint>
#include <limits>
#include <memory>

namespace inviwo {

const ProcessorInfo ParticlePathlinesToMesh::processorInfo_{
    "org.inviwo.ParticlePathlinesToMesh",  // Class identifier
    "Particle Pathlines To Mesh",          // Display name
    "Mesh Creation",                       // Category
    CodeState::Experimental,               // Code state
    Tags::GL,                             // Tags
    R"(Creates indexed line geometry from GPU particle pathline history.

The trajectory position buffer is shared with the output mesh and is not copied.
The index buffer describes the valid pathline segments in chronological order,
including circular-buffer wraparound and per-particle reseeding.)"_unindentHelp};

const ProcessorInfo& ParticlePathlinesToMesh::getProcessorInfo() const { return processorInfo_; }

ParticlePathlinesToMesh::ParticlePathlinesToMesh()
    : Processor{}
    , pathlines_{"pathlines", "GPU particle pathline history"_help}
    , mesh_{"mesh", "Indexed particle pathline geometry"_help}
    , liveMesh_{"liveMesh", "Live pathline endpoint segments"_help}
    , fadeFraction_{"fadeFraction", "Fade Fraction", 0.35f, 0.0f, 1.0f}
    , fadeExponent_{"fadeExponent", "Fade Exponent", 2.0f, 0.1f, 8.0f}
    , indices_{std::make_shared<IndexBuffer>()}
    , fadedColors_{std::make_shared<Buffer<vec4>>()}
    , livePositions_{std::make_shared<Buffer<vec4>>()}
    , liveColors_{std::make_shared<Buffer<vec4>>()}
    , fadeShader_ {{{
            ShaderType::Compute, std::string {"particlepathlinesfade.comp"}}},
            Shader::Build::No}
    {

    addPort(pathlines_);
    addPort(mesh_);
    addPort(liveMesh_);

    addProperties(fadeFraction_, fadeExponent_);

    fadeShader_.onReload([this]() { invalidate(InvalidationLevel::InvalidResources); });
}

void ParticlePathlinesToMesh::initializeResources() { fadeShader_.build(); }

void ParticlePathlinesToMesh::process() {
    const auto pathlines = pathlines_.getData();
    const auto particleCount = pathlines->getParticleCount();
    const auto trailLength = pathlines->getTrailLength();
    const auto trailHead = pathlines->getTrailHead();

    const auto& positions = pathlines->getPositions();
    const auto& validCounts = pathlines->getValidCounts();
    const auto& trailColors = pathlines->getTrailColor();
    const auto& currentPositions = pathlines->getCurrentPositions();
    const auto& currentColors = pathlines->getCurrentColors();

    const bool hasCurrentData = currentPositions && currentColors &&
                                currentPositions->getSize() >= particleCount &&
                                currentColors->getSize() >= particleCount;
    if (!positions || !validCounts || !trailColors ||  particleCount == 0 || trailLength < 2) {
        auto emptyMesh = std::make_shared<Mesh>(DrawType::Lines, ConnectivityType::None);

        if (positions) {
            emptyMesh->addBuffer(BufferType::PositionAttrib, positions);
        }
        if (trailColors) {
            emptyMesh->addBuffer(BufferType::ColorAttrib, trailColors);
        }

        mesh_.setData(emptyMesh);
        auto emptyLiveMesh = std::make_shared<Mesh>(DrawType::Lines, ConnectivityType::None);

        liveMesh_.setData(emptyLiveMesh);
        return;
    }

    if (particleCount > std::numeric_limits<size_t>::max() / trailLength) {

        throw Exception("ParticlePathlines sample count overflow");
    }
    const auto expectedPositionCount = particleCount * trailLength;
    if (positions->getSize() < expectedPositionCount) {
        throw Exception(
            "ParticlePathlines position buffer is smaller than "
            "particleCount * trailLength");
    }
    if (validCounts->getSize() < particleCount) {
        throw Exception("ParticlePathlines valid-count buffer is smaller than particleCount");
    }
    if (trailColors->getSize() < expectedPositionCount) {
        throw Exception(
            "ParticlePathlines color buffer is smaller than "
            "particleCount * trailLength");
    }
    if (expectedPositionCount > static_cast<size_t>(std::numeric_limits<uint32_t>::max())) {
        throw Exception("ParticlePathlines contains too many positions for a uint32 index buffer");
    }

    const auto maxIndexCount = particleCount * (trailLength - 1) * size_t{2};

    if (!fadedColors_ || fadedColors_->getSize() != expectedPositionCount) {

        fadedColors_ = std::make_shared<Buffer<vec4>>(expectedPositionCount, BufferUsage::Dynamic);
    }

    if (!indices_ || indices_->getSize() != maxIndexCount) {

        indices_ = std::make_shared<IndexBuffer>(maxIndexCount, BufferUsage::Dynamic);
    }

    size_t liveVertexCount = 0;

    if (hasCurrentData) {
        if (particleCount > std::numeric_limits<size_t>::max() / size_t{2}) {

            throw Exception("ParticlePathlines live vertex count overflow");
        }

        liveVertexCount = particleCount * size_t{2};

        if (!livePositions_ || livePositions_->getSize() != liveVertexCount) {

            livePositions_ = std::make_shared<Buffer<vec4>>(liveVertexCount, BufferUsage::Dynamic);
        }

        if (!liveColors_ || liveColors_->getSize() != liveVertexCount) {

            liveColors_ = std::make_shared<Buffer<vec4>>(liveVertexCount, BufferUsage::Dynamic);
        }
    }

    const auto trailColorsGL = trailColors->getRepresentation<BufferGL>();
    const auto validCountsGL = validCounts->getRepresentation<BufferGL>();
    const auto trailPositionsGL = positions->getRepresentation<BufferGL>();
    auto fadedColorsGL = fadedColors_->getEditableRepresentation<BufferGL>();
    auto indicesGL = indices_->getEditableRepresentation<BufferGL>();

    const BufferGL* currentPositionsGL = nullptr;
    const BufferGL* currentColorsGL = nullptr;

    BufferGL* livePositionsGL = nullptr;
    BufferGL* liveColorsGL = nullptr;

    if (hasCurrentData) {
        currentPositionsGL = currentPositions->getRepresentation<BufferGL>();

        currentColorsGL = currentColors->getRepresentation<BufferGL>();

        livePositionsGL = livePositions_->getEditableRepresentation<BufferGL>();

        liveColorsGL = liveColors_->getEditableRepresentation<BufferGL>();
    }

    fadeShader_.activate();
    fadeShader_.setUniform("particleCount", static_cast<unsigned int>(particleCount));
    fadeShader_.setUniform("trailLength", static_cast<unsigned int>(trailLength));
    fadeShader_.setUniform("trailHead", static_cast<unsigned int>(trailHead));
    fadeShader_.setUniform("fadeFraction", fadeFraction_.get());
    fadeShader_.setUniform("fadeExponent", fadeExponent_.get());
    fadeShader_.setUniform("generateLive", hasCurrentData);

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, trailColorsGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, validCountsGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, fadedColorsGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, indicesGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, trailPositionsGL->getId());

    if (hasCurrentData) {
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, currentPositionsGL->getId());

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 6, currentColorsGL->getId());

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 7, livePositionsGL->getId());

        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 8, liveColorsGL->getId());
    }

    constexpr size_t localSize = 256;
    const auto workGroupCount =
        static_cast<GLuint>((expectedPositionCount + localSize - 1) / localSize);

    glDispatchCompute(workGroupCount, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT |
                    GL_ELEMENT_ARRAY_BARRIER_BIT);

    for (GLuint binding = 0; binding <= 4; ++binding) {
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, 0);
    }

    if (hasCurrentData) {
        for (GLuint binding = 5; binding <= 8; ++binding) {
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, 0);
        }
    }
    fadeShader_.deactivate();

    auto mesh = std::make_shared<Mesh>(DrawType::Lines, ConnectivityType::None);
    mesh->addBuffer(BufferType::PositionAttrib, positions);
    mesh->addBuffer(BufferType::ColorAttrib, fadedColors_);
    mesh->addIndices(Mesh::MeshInfo{DrawType::Lines, ConnectivityType::None}, indices_);
    mesh_.setData(mesh);
    if (hasCurrentData) {
        auto liveMesh = std::make_shared<Mesh>(DrawType::Lines, ConnectivityType::None);

        liveMesh->addBuffer(BufferType::PositionAttrib, livePositions_);

        liveMesh->addBuffer(BufferType::ColorAttrib, liveColors_);

        liveMesh_.setData(liveMesh);

    } else {
        auto emptyLiveMesh = std::make_shared<Mesh>(DrawType::Lines, ConnectivityType::None);

        liveMesh_.setData(emptyLiveMesh);
    }
}

}  // namespace inviwo
