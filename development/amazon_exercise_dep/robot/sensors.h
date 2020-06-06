
//#include <Ice/Ice.h>
//#include <IceUtil/IceUtil.h>

#include </opt/jderobot/include/jderobot/types/laserData.h>
#include </opt/jderobot/include/jderobot/types/image.h>
#include </opt/jderobot/include/jderobot/types/pose3d.h>

//Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../comm/communicator.hpp"
#include "../comm/laserClient.hpp"
#include "../comm/cameraClient.hpp"
#include "../comm/pose3dClient.hpp"


class Sensors
{
public:
    Sensors(Comm::Communicator* jdrc);

    JdeRobotTypes::Pose3d getPose();
    JdeRobotTypes::LaserData getLaserData();

    JdeRobotTypes::Image getImage1();
    JdeRobotTypes::Image getImage2();


private:

    Comm::Communicator* jdrc;

    //LASER DATA
    Comm::LaserClient* laserClient;

    Comm::CameraClient* camera1;
    Comm::CameraClient* camera2;

    Comm::Pose3dClient* poseClient;

};
