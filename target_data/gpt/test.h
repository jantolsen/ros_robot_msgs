
// Target ROS Messages
#include "target_data/TargetData.h"
#include "target_data/TargetJoint.h"
#include "target_data/TargetCartesian.h"
#include "target_data/TargetJointExtAxis.h"

namespace Target
{
    enum TargetType
    {
        JOINT,
        CARTESIAN
    };


    class TargetInterface
    {
        public:
            virtual target_data::TargetData getTargetData();

            virtual void updateTargetData(
                    target_data::TargetData target_data);

            virtual void printTargetData() = 0;

        protected:
            static const std::string CLASS_PREFIX;

            target_data::TargetData target_data_;
    };


    class TargetCartesian : public TargetInterface
    {
        public:

        protected:
            TargetData::TargetCartesian loadParamTargetCartesian(
                const XmlRpc::XmlRpcValue& param_xml)
    };


    class TargetJoint : public TargetInterface
    {
        public:

        protected:
            TargetData::TargetJoint loadParamTargetJoint(
                const XmlRpc::XmlRpcValue& param_xml)
    };


    class TargetContext
    {
        public:

        protected:

            TargetData::TargetData loadParam(
                const XmlRpc::XmlRpcValue& param_xml)

            TargetInterface* targetInterface_;
            std::shared_ptr<TargetInterface> targetInterface_;
            std::unique_ptr<TargetInterface> targetInterface_;

            std::map<std::string, TargetType> target_type_map_;
            std::vector<std::string> target_type_names_vec_;
    };

}