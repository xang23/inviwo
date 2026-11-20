/*********************************************************************************
*
* Inviwo - Interactive Visualization Workshop
*
* Copyright (c) 2016-2026 Inviwo Foundation
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

#pragma once

#include <modules/vectorfieldvisualizationgl/vectorfieldvisualizationglmoduledefine.h>
#include <modules/vectorfieldvisualizationgl/datastructures/particlepathlines.h>
#include <modules/opengl/shader/shader.h>

#include <inviwo/core/datastructures/buffer/buffer.h>
#include <inviwo/core/ports/meshport.h>
#include <inviwo/core/processors/processor.h>
#include <inviwo/core/processors/processorinfo.h>
#include <inviwo/core/properties/ordinalproperty.h>

#include <memory>

namespace inviwo {
class IVW_MODULE_VECTORFIELDVISUALIZATIONGL_API ParticlePathlinesToMesh : public Processor {
public:
    ParticlePathlinesToMesh();
    virtual ~ParticlePathlinesToMesh() = default;

    virtual void process() override;
    virtual void initializeResources() override;

    virtual const ProcessorInfo& getProcessorInfo() const override;
    static const ProcessorInfo processorInfo_;

private:
    ParticlePathlinesInport pathlines_;
    MeshOutport mesh_;
    MeshOutport liveMesh_;
    FloatProperty fadeFraction_;
    FloatProperty fadeExponent_;

    std::shared_ptr<IndexBuffer> indices_;
    std::shared_ptr<Buffer<vec4>> fadedColors_;
    std::shared_ptr<Buffer<vec4>> livePositions_;
    std::shared_ptr<Buffer<vec4>> liveColors_;

    Shader fadeShader_;
};

}  // namespace inviwo
