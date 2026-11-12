#pragma once

#include <modules/vectorfieldvisualizationgl/vectorfieldvisualizationglmoduledefine.h>

#include <inviwo/core/datastructures/buffer/buffer.h>
#include <inviwo/core/datastructures/datatraits.h>
#include <inviwo/core/ports/datainport.h>
#include <inviwo/core/ports/dataoutport.h>
#include <inviwo/core/util/document.h>
#include <inviwo/core/util/glmvec.h>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <sstream>
#include <utility>

namespace inviwo {

class IVW_MODULE_VECTORFIELDVISUALIZATIONGL_API ParticlePathlines {
public:
    ParticlePathlines(std::shared_ptr<Buffer<vec4>> positions,
                      std::shared_ptr<Buffer<uint32_t>> validCounts, size_t particleCount,
                      size_t trailLength, size_t trailHead)
        : positions_{std::move(positions)}
        , validCounts_{std::move(validCounts)}
        , particleCount_{particleCount}
        , trailLength_{trailLength}
        , trailHead_{trailHead} {}

    const std::shared_ptr<Buffer<vec4>>& getPositions() const { return positions_; }

    const std::shared_ptr<Buffer<uint32_t>>& getValidCounts() const { return validCounts_; }

    size_t getParticleCount() const { return particleCount_; }

    size_t getTrailLength() const { return trailLength_; }

    size_t getTrailHead() const { return trailHead_; }

private:
    std::shared_ptr<Buffer<vec4>> positions_;
    std::shared_ptr<Buffer<uint32_t>> validCounts_;

    size_t particleCount_;
    size_t trailLength_;
    size_t trailHead_;
};

template <>
struct DataTraits<ParticlePathlines> {
    static constexpr std::string_view classIdentifier() { return "org.inviwo.ParticlePathlines"; }

    static constexpr std::string_view dataName() { return "Particle Pathlines"; }

    static constexpr uvec3 colorCode() {
        // Deliberately similar to IntegralLineSet for trajectory-like data
        return {255, 150, 0};
    }

    static Document info(const ParticlePathlines& data) {
        std::ostringstream oss;

        oss << "Particle Pathlines with " << data.getParticleCount() << " particles, trail length "
            << data.getTrailLength() << ", trail head " << data.getTrailHead();

        Document doc;
        doc.append("p", oss.str());
        return doc;
    }
};

using ParticlePathlinesInport = DataInport<ParticlePathlines>;

using ParticlePathlinesOutport = DataOutport<ParticlePathlines>;

}  // namespace inviwo
