// -*-c++-*---------------------------------------------------------------------------------------
// Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <yaml-cpp/yaml.h>

#include <tagslam/logging.hpp>
#include <tagslam/simple_body.hpp>
#include <tagslam/yaml.hpp>
#include <tagslam/yaml_utils.hpp>

namespace tagslam
{
static rclcpp::Logger get_logger()
{
  return (rclcpp::get_logger("simple_body"));
}

bool SimpleBody::parse(const YAML::Node & body, const BodyPtr & bp)
{
  if (body["tags"]) {
    TagVec tv = Tag::parseTags(body["tags"], defaultTagSize_, bp);
    addTags(tv);
  }
  if (body["tag_sizes"]) {
    const auto tagSizes = body["tag_sizes"];
    if (!tagSizes.IsSequence()) {
      BOMB_OUT("tag_sizes must be a list!");
    }
    for (const auto & tagSize : tagSizes) {
      const int id = yaml::parse<int>(tagSize, "id");
      const int bits = yaml::parse<int>(tagSize, "bits", 6);
      const double size = yaml::parse<double>(tagSize, "size", defaultTagSize_);
      if (tags_.find(id) != tags_.end()) {
        LOG_WARN(
          "tag size entry ignored for tag " << id
                                             << " because tag is already set");
        continue;
      }
      addTag(Tag::make(id, bits, size, PoseWithNoise(), bp));
    }
  }
  return (true);
}

bool SimpleBody::write(std::ostream & os, const std::string & prefix) const
{
  // write common section
  if (!Body::writeCommon(os, prefix)) {
    return (false);
  }
  return (true);
}

}  // namespace tagslam
