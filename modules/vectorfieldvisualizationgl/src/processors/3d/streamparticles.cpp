/*********************************************************************************
 *
 * Inviwo - Interactive Visualization Workshop
 *
 * Copyright (c) 2019-2026 Inviwo Foundation
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

#include <modules/vectorfieldvisualizationgl/processors/3d/streamparticles.h>

#include <inviwo/core/common/inviwoapplication.h>
#include <inviwo/core/datastructures/buffer/buffer.h>
#include <inviwo/core/datastructures/buffer/bufferramprecision.h>
#include <inviwo/core/datastructures/coordinatetransformer.h>
#include <inviwo/core/datastructures/geometry/geometrytype.h>
#include <inviwo/core/datastructures/geometry/mesh.h>
#include <inviwo/core/datastructures/representationconverter.h>
#include <inviwo/core/datastructures/representationconverterfactory.h>
#include <inviwo/core/datastructures/transferfunction.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/ports/volumeport.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/processors/processorinfo.h>
#include <inviwo/core/processors/processorstate.h>
#include <inviwo/core/processors/processortags.h>
#include <inviwo/core/properties/invalidationlevel.h>
#include <inviwo/core/properties/minmaxproperty.h>
#include <inviwo/core/properties/optionproperty.h>
#include <inviwo/core/properties/ordinalproperty.h>
#include <inviwo/core/properties/propertysemantics.h>
#include <inviwo/core/properties/transferfunctionproperty.h>
#include <inviwo/core/util/clock.h>
#include <inviwo/core/util/exception.h>
#include <inviwo/core/util/glmutils.h>
#include <inviwo/core/util/glmvec.h>
#include <inviwo/core/util/logcentral.h>
#include <inviwo/core/util/pathtype.h>
#include <inviwo/core/util/rendercontext.h>
#include <inviwo/core/util/staticstring.h>
#include <inviwo/core/util/stringconversion.h>
#include <inviwo/core/util/timer.h>
#include <inviwo/core/util/zip.h>
#include <inviwo/core/util/filesystem.h>
#include <modules/opengl/buffer/buffergl.h>
#include <modules/opengl/inviwoopengl.h>
#include <modules/opengl/openglcapabilities.h>
#include <modules/opengl/shader/shader.h>
#include <modules/opengl/shader/shaderobject.h>
#include <modules/opengl/shader/shadertype.h>
#include <modules/opengl/shader/shaderutils.h>
#include <modules/opengl/texture/textureunit.h>
#include <modules/opengl/texture/textureutils.h>
#include <modules/opengl/volume/volumeutils.h>
#include <modules/vectorfieldvisualization/ports/seedpointsport.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <random>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <cmath>

#include <glm/detail/qualifier.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec2.hpp>
#include <glm/vec4.hpp>

namespace inviwo {

// The Class Identifier has to be globally unique. Use a reverse DNS naming scheme
const ProcessorInfo StreamParticles::processorInfo_{
    "org.inviwo.StreamParticles",  // Class identifier
    "Stream Particles",            // Display name
    "Particle",                    // Category
    CodeState::Experimental,       // Code state
    Tags::GL,                      // Tags
    R"(Processor that simulate particles moving through a velocity filed. Particles are initialized
    based on the __seeds__ inport and are advected through the field using the Velocity Volume on
    the __volume__ inport. If particles velocity has been zero for 0.5 seconds their position will
    be reset to a random element in the seedpoint input vector. Reseeding is done on a less frequent
    intervall than the advection based on the __Reseed interval__ property. Using GLSL Compute
    Shaders, requires OpenGL 4.3)"_unindentHelp};

const ProcessorInfo& StreamParticles::getProcessorInfo() const { return processorInfo_; }

StreamParticles::StreamParticles()
    : Processor()
    , volume_{"volume", "Velocity field"_help}
    , seeds_{"seeds", "Starting position for particles"_help}
    , meshPort_{"particles",
                "Mesh containing position, color and radii buffers. Can be "
                "connected to (for example) the Sphere Renderer Processor to render them"_help}
    , pathlinesPort_{"pathlines", "Particle pathline history"_help}
    , seedingSpace_{"seedingSpace",
                    "Seeding Space",
                    "Tells the processor which space the seeds are defined in"_help,
                    {{"data", "Data", SeedingSpace::Data}, {"world", "World", SeedingSpace::World}}}
    , advectionSpeed_{"advectionSpeed", "Advection Speed",
                      util::ordinalLength(0.01f, 1.0f)
                          .set("How fast the particles will move each advection."_help)}
    , internalSteps_{"advectionsPerFrame", "Advections per Frame",
                     util::ordinalCount<int>(10, 100)
                         .setMin(1)
                         .set(InvalidationLevel::InvalidResources)
                         .set("Can be used to increase advection "
                              "speed without loss of precision"_help)}
    , particleSize_{"particleSize",
                    "Particle radius",
                    "Maps velocity to radius of particle (For rendering that supports "
                    "the Radii buffer, e.g. Sphere Renderer)."_help,
                    0.025f,
                    0.035f,
                    0.0f,
                    1.0f}
    , minV_{"minV", "Min velocity",
            util::ordinalLength(0.0f)
                .set("Used together with __Max velocity__ to map velocity magnitude "
                     "from [Min velocity, Max velocity] -> [0 1], used for mapping "
                     "velocity to radius and color"_help)
                .set(InvalidationLevel::Valid)
                .set(PropertySemantics::Text)}
    , maxV_{"maxV", "Max velocity",
            util::ordinalLength(1.0f)
                .set("Used together with __Min velocity__ to map velocity magnitude "
                     "from [Min velocity, Max velocity] -> [0 1], used for mapping "
                     "velocity to radius and color"_help)
                .set(InvalidationLevel::Valid)
                .set(PropertySemantics::Text)}
    , tf_{"tf", "Velocity mapping", "Transferfunction to map a velocity to color"_help,
          TransferFunction::load(
              filesystem::getPath(PathType::TransferFunctions, "/matplotlib/plasma.itf"))}
    , reseedInterval_{"reseedsInterval", "Reseed interval",
                      util::ordinalLength(1.0f, 10.f)
                          .set("Seconds between reseeding. When reseeding particles whose "
                               "life is zero will get new position by selecting (randomly) "
                               "from the input seed vector"_help)}
    , running_{"running", "Run Simulation", true}
    , resetSimulation_{"resetSimulation", "Reset Simulation"}
    , trailSampleInterval_{"trailSampleInterval", "Trail Sample Interval", 0.05f, 0.001f, 1.0f}
    , trailLength_{"trailLength", "Trail Length", 30, 2, 200}
    , shader_{{{ShaderType::Compute, std::string{"streamparticles.comp"}}}, Shader::Build::No}
    , timer_{Timer::Milliseconds(17), [&]() { update(); }}
    , reseedtime_{0.0}
    , prevT_{0}
    , clock_{}
    , simulationTime_{0.0}
    , trailSampleAccumulator_{0.0}
    , writeTrailSample_{false}
    , trailHead_{0}
    , ready_{false}
    , buffersDirty_{true} {

    addPort(volume_);
    addPort(seeds_);
    addPort(meshPort_);
    addPort(pathlinesPort_);

    addProperties(seedingSpace_, advectionSpeed_, internalSteps_, particleSize_, minV_, maxV_, tf_,
                  reseedInterval_, running_, resetSimulation_, trailSampleInterval_, trailLength_);

    shader_.onReload([this]() {
        invalidate(InvalidationLevel::InvalidOutput);
        buffersDirty_ = true;
    });

    resetSimulation_.onChange([this]() {
        resetSimulation();
        invalidate(InvalidationLevel::InvalidOutput);
    });

    trailLength_.onChange([this]() { buffersDirty_ = true; });

    timer_.start();
}

void StreamParticles::initializeResources() {
    if (OpenGLCapabilities::getOpenGLVersion() < 430) {
        isReady_.setUpdate([]() -> ProcessorStatus {
            static constexpr std::string_view error{
                "The StreamParticles processor requires OpenGL 4.3"};
            return {ProcessorStatus::Error, error};
        });
        throw Exception(SourceContext{}, "The StreamParticles processor requires OpenGL 4.3");
    }

    shader_.getShaderObject(ShaderType::Compute)
        ->setShaderDefine("NUM_ADVECTIONS", true, fmt::to_string(internalSteps_.get()));
    shader_.build();
}

void StreamParticles::process() {
    if (seeds_.isChanged() || seedingSpace_.isModified() || buffersDirty_) {
        initBuffers();
    }

    if (running_.get()) {
        reseed();
        advect();
    } else {
        // Prevent a large timestep and immediate reseed after resuming.
        const auto currentTime = clock_.getElapsedSeconds();
        prevT_ = currentTime;
        reseedtime_ = currentTime;

        writeTrailSample_ = false;
    }

    meshPort_.setData(mesh_);
    pathlinesPort_.setData(
        std::make_shared<ParticlePathlines>(bufTrailPos_, bufTrailCount_, seeds_.getData()->size(),
                                            static_cast<size_t>(trailLength_.get()), trailHead_));
    ready_ = true;
}

void StreamParticles::update() {
    try {
        RenderContext::getPtr()->activateDefaultRenderContext();
        if (ready_) {
            ready_ = false;
            invalidate(InvalidationLevel::InvalidOutput);
        }
    } catch (const Exception& e) {
        log::exception(e);
    }
}

void StreamParticles::initBuffers() {
    const auto seeds = seeds_.getData();
    const auto numPoints = seeds->size();

    bufPos_ = std::make_shared<Buffer<vec4>>(numPoints, BufferUsage::Dynamic);
    bufLife_ = std::make_shared<Buffer<float>>(numPoints, BufferUsage::Dynamic);
    bufRad_ = std::make_shared<Buffer<float>>(numPoints, BufferUsage::Dynamic);
    bufCol_ = std::make_shared<Buffer<vec4>>(numPoints, BufferUsage::Dynamic);

    const auto trailLength = static_cast<size_t>(trailLength_.get());
    if (numPoints > 0 && trailLength > std::numeric_limits<size_t>::max() / numPoints) {
        throw Exception(SourceContext{},
                        "Particle trail buffer size overflow: {} particles times {} samples",
                        numPoints, trailLength);
    }

    const auto numTrailPositions = numPoints * trailLength;

    bufTrailPos_ = std::make_shared<Buffer<vec4>>(numTrailPositions, BufferUsage::Dynamic);
    bufTrailCount_ = std::make_shared<Buffer<uint32_t>>(numPoints, BufferUsage::Dynamic);


    std::mt19937 rand;
    std::uniform_real_distribution<float> dist(0, 1);

    auto& positions = bufPos_->getEditableRAMRepresentation()->getDataContainer();
    auto& lifes = bufLife_->getEditableRAMRepresentation()->getDataContainer();
    auto& trailPositions = bufTrailPos_->getEditableRAMRepresentation()->getDataContainer();

    auto& trailCounts = bufTrailCount_->getEditableRAMRepresentation()->getDataContainer();

    std::fill(trailPositions.begin(), trailPositions.end(), vec4{0.0f});
    std::fill(trailCounts.begin(), trailCounts.end(), uint32_t{0});

    trailHead_ = 0;

    if (seedingSpace_.get() == SeedingSpace::World) {
        std::transform(seeds->begin(), seeds->end(), positions.begin(),
                       [](vec3 seed) { return vec4(seed, 1.0f); });
    } else {
        std::transform(
            seeds->begin(), seeds->end(), positions.begin(),
            [toWorld =
                 volume_.getData()->getCoordinateTransformer().getDataToWorldMatrix()](vec3 seed) {
                const vec4 p = toWorld * vec4(seed, 1.0);
                return p / p.w;
            });
    }

    std::ranges::fill(lifes, 1.0f);

    bufPos_->getEditableRepresentation<BufferGL>();
    bufLife_->getEditableRepresentation<BufferGL>();
    bufRad_->getEditableRepresentation<BufferGL>();
    bufCol_->getEditableRepresentation<BufferGL>();
    bufTrailPos_->getEditableRepresentation<BufferGL>();
    bufTrailCount_->getEditableRepresentation<BufferGL>();

    mesh_ = std::make_shared<Mesh>();
    mesh_->addBuffer(BufferType::PositionAttrib, bufPos_);
    mesh_->addBuffer(BufferType::RadiiAttrib, bufRad_);
    mesh_->addBuffer(BufferType::ColorAttrib, bufCol_);

    prevT_ = reseedtime_ = clock_.getElapsedSeconds();
    buffersDirty_ = false;
}

void StreamParticles::reseed() {
    const auto curT = clock_.getElapsedSeconds();
    if (curT >= reseedtime_ + reseedInterval_.get()) {
        const auto seeds = seeds_.getData();

        auto& positions = bufPos_->getEditableRAMRepresentation()->getDataContainer();
        auto& lifes = bufLife_->getEditableRAMRepresentation()->getDataContainer();
        auto& trailCounts = bufTrailCount_->getEditableRAMRepresentation()->getDataContainer();
        if (seeds->empty()) {
            return;
        }

        std::mt19937 rand;
        std::uniform_int_distribution<size_t> dist(0, seeds->size() - 1);

        const auto seedTransform =
            [space = seedingSpace_.get(),
             toWorld = volume_.getData()->getCoordinateTransformer().getDataToWorldMatrix()](
                vec3 seed) -> vec4 {
            if (space == SeedingSpace::World) {
                return {seed, 1.0f};
            } else {
                const vec4 p = toWorld * vec4(seed, 1.0f);
                return p / p.w;
            }
        };

        for (const auto z : util::zip(positions, lifes, trailCounts)) {
            auto& position = get<0>(z);
            auto& life = get<1>(z);
            auto& trailCount = get<2>(z);

            if (life <= 0.0f){
                position = seedTransform((*seeds)[dist(rand)]);
                life = 1.0f;
                trailCount = 0u;
            }
        }

        reseedtime_ = curT;
    }
}

void StreamParticles::advect() {
    TextureUnitContainer cont;

    const auto t = clock_.getElapsedSeconds();
    const auto dt = t - prevT_;
    prevT_ = t;

    const float clampedDt = std::min(static_cast<float>(dt), 0.1f);
    updateTrailSampling(clampedDt);

    const auto volume = volume_.getData();

    auto bufPosGL = bufPos_->getEditableRepresentation<BufferGL>();
    auto bufLifeGL = bufLife_->getEditableRepresentation<BufferGL>();
    auto bufRadGL = bufRad_->getEditableRepresentation<BufferGL>();
    auto bufColGL = bufCol_->getEditableRepresentation<BufferGL>();
    auto bufTrailPosGL = bufTrailPos_->getEditableRepresentation<BufferGL>();
    auto bufTrailCountGL = bufTrailCount_->getEditableRepresentation<BufferGL>();

    shader_.activate();

    shader_.setUniform("dt", clampedDt);
    utilgl::setUniforms(shader_, advectionSpeed_, internalSteps_, minV_, maxV_);
    utilgl::bindAndSetUniforms(shader_, cont, tf_);

    shader_.setUniform("minR", particleSize_.get().x);
    shader_.setUniform("maxR", particleSize_.get().y);
    shader_.setUniform("toTextureMatrix",
                       volume->getCoordinateTransformer().getWorldToTextureMatrix());

    shader_.setUniform("trailLength", static_cast<unsigned int>(trailLength_.get()));
    shader_.setUniform("trailHead", static_cast<unsigned int>(trailHead_));
    shader_.setUniform("writeTrailSample", writeTrailSample_);

    utilgl::bindAndSetUniforms(shader_, cont, *volume, "velocityField");

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, bufPosGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, bufLifeGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, bufRadGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, bufColGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, bufTrailPosGL->getId());
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, bufTrailCountGL->getId());

    glDispatchCompute(static_cast<GLuint>(seeds_.getData()->size()), 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    glMemoryBarrier(GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);

    if (writeTrailSample_) {
        trailHead_ = (trailHead_ + 1) % static_cast<size_t>(trailLength_.get());
    }
    shader_.deactivate();

    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, 0);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, 0);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, 0);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, 0);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, 0);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, 0);
}

void StreamParticles::resetSimulation() {
    simulationTime_ = 0.0;
    trailSampleAccumulator_ = 0.0;
    writeTrailSample_ = false;
    trailHead_ = 0;

    const auto currentTime = clock_.getElapsedSeconds();

    prevT_ = currentTime;
    reseedtime_ = currentTime;

    buffersDirty_ = true;
    ready_ = false;
}

void StreamParticles::updateTrailSampling(float dt) {
    writeTrailSample_ = false;

    simulationTime_ += static_cast<double>(dt);
    trailSampleAccumulator_ += static_cast<double>(dt);

    const double sampleInterval = static_cast<double>(trailSampleInterval_.get());

    if (trailSampleAccumulator_ >= sampleInterval) {
        writeTrailSample_ = true;

        trailSampleAccumulator_ = std::fmod(trailSampleAccumulator_, sampleInterval);
    }
}

}  // namespace inviwo
