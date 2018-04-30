#ifndef INCLUDE_IK_BASE_H_
#define INCLUDE_IK_BASE_H_

#include <kdl/frames.hpp>

namespace ik_base
{
  class IKPlugin
  {
    public:
      virtual bool initialize(const std::vector<double> &min_angles,
      												const std::vector<double> &max_angles) = 0;
      virtual int ik(const KDL::Frame &frame, std::vector<double> &joints) = 0;
      virtual ~IKPlugin(){}

    protected:
      IKPlugin(){}
  };
};
#endif