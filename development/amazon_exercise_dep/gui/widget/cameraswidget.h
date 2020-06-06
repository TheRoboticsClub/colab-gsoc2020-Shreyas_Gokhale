#ifndef CAMERASWIDGET_H
#define CAMERASWIDGET_H

#include <QtWidgets>
#include </opt/jderobot/include/jderobot/types/image.h>

#include "../../robot/robot.h"

class CamerasWidget: public QWidget
{
    Q_OBJECT

public:
    CamerasWidget(Robot *robot);

    void update();

private:
    Robot* robot;
    QLabel* labelImage1;
    QLabel* labelImage2;

protected:
    void mousePressEvent(QMouseEvent* event);
public slots:



};

#endif // CAMERASWIDGET_H
