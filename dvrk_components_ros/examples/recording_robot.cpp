class QProcess;
class QObject;

#include <QProcess>
#include <QObject>
#include <iostream>

int main(int argc, char **argv)
{
    QObject *parent;
    QProcess *QProcess_Recording = new QProcess(parent);

    //QProcess_Recording->setProgram("roslaunch dvrk_robot dvrk_camera_console_record.launch");
    //QProcess_Recording->setProgram("/home/arclab/catkin_ws/devel_release/.private/dvrk_components_ros/lib/dvrk_components_ros/csvwriter_psm ~");
    //QProcess_Recording->startDetached("/home/arclab/catkin_ws/devel_release/.private/dvrk_components_ros/lib/dvrk_components_ros/csvwriter_psm ~");
    QProcess_Recording->startDetached("roslaunch dvrk_robot dvrk_camera_console_record.launch");

    if (!QProcess_Recording->waitForStarted()){
        std::cout << "Failed to Record" << std::endl;
        return 0;
    }


    if (!QProcess_Recording->waitForFinished()){
        std::cout << "Finished Recording" << std::endl;
        return 0;
    }


}
