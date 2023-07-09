#include "my_inverKine.h"

Position forwardKinematics(DHParameters* dhParams, JointAngles* jointAngles) 
{
    Position pos;
    double c1, c2, c3, c4, c5, c6;
    double s1, s2, s3, s4, s5, s6;

    c1 = cos(jointAngles->joint1);
    c2 = cos(jointAngles->joint2);
    c3 = cos(jointAngles->joint3);
    c4 = cos(jointAngles->joint4);
    c5 = cos(jointAngles->joint5);
    c6 = cos(jointAngles->joint6);

    s1 = sin(jointAngles->joint1);
    s2 = sin(jointAngles->joint2);
    s3 = sin(jointAngles->joint3);
    s4 = sin(jointAngles->joint4);
    s5 = sin(jointAngles->joint5);
    s6 = sin(jointAngles->joint6);

    // 计算末端坐标
    pos.x = dhParams[0].a * c1 * (c2 * c3 * c4 * c5 * c6 - c2 * s3 * s4 * s5 * s6 + s2 * s4 * s6)
            - dhParams[0].a * s1 * (c2 * s3 * s5 - c2 * c3 * c4 * c5 + c2 * s4 * s5) + dhParams[1].a * c1 * c2 * c3
            - dhParams[1].a * s1 * s2 + dhParams[2].a * c1 * (c2 * c3 * c4 * c5 + c2 * s3 * s4 * s5)
            + dhParams[2].a * s1 * (c2 * s3 * s4 * s5 - c2 * c3 * c4 * c5 - c2 * s4 * s5)
            - dhParams[3].a * s1 * c3 * s4 * s6 + dhParams[3].a * s1 * c2 * c4 * c6
            + dhParams[3].a * c1 * c3 * s5 * s6 + dhParams[4].a * c1 * s2 * c4 * c6
            + dhParams[4].a * c1 * c2 * s3 * s5 * c6 - dhParams[4].a * c1 * c2 * c3 * c5 * s6
            + dhParams[5].a * c1 * s2 * s4 * s6 + dhParams[5].a * c1 * c2 * s3 * s5 * s6
            + dhParams[5].a * c1 * c2 * c3 * c5 * c6;

    pos.y = dhParams[0].a * s1 * (c2 * c3 * c4 * c5 * c6 - c2 * s3 * s4 * s5 * s6 + s2 * s4 * s6)
            + dhParams[0].a * c1 * (c2 * s3 * s5 - c2 * c3 * c4 * c5 + c2 * s4 * s5) + dhParams[1].a * s1 * c2 * c3
            + dhParams[1].a * c1 * s2 + dhParams[2].a * s1 * (c2 * c3 * c4 * c5 + c2 * s3 * s4 * s5)
            + dhParams[2].a * c1 * (c2 * s3 * s4 * s5 - c2 * c3 * c4 * c5 - c2 * s4 * s5)
            - dhParams[3].a * c1 * s3 * s5 * s6 + dhParams[3].a * c1 * c2 * c4 * c5
            + dhParams[3].a * s1 * c3 * s4 * c6 - dhParams[4].a * s1 * s2 * c4 * s6
            + dhParams[4].a * c1 * c3 * c5 * s6 + dhParams[4].a * c1 * c2 * s3 * s5 * s6
            + dhParams[4].a * c1 * c2 * c3 * c5 * c6 - dhParams[5].a * s1 * s2 * s4 * s6
            + dhParams[5].a * c1 * c3 * c5 * s6 + dhParams[5].a * c1 * c2 * s3 * s5 * s6
            + dhParams[5].a * c1 * c2 * c3 * c5 * c6;

    pos.z = dhParams[0].d + dhParams[1].d + dhParams[2].d
            + dhParams[3].d * (c2 * c4 * c5 - s2 * s4) + dhParams[4].d * (c2 * c3 * s5 + s2 * c4 * s5)
            + dhParams[5].d * (c2 * s3 * c5 - s2 * c3 * s5);

    return pos;
}

int inverseKinematics(DHParameters* dhParams, Position* pos, Orientation* ori, JointAngles* jointAngles) 
{
    double r1, c3, s3, c2, s2;
	
	// 计算关节1的角度
    jointAngles->joint1 = atan2(pos->y, pos->x);

    // 计算末端到关节1的距离
    r1 = sqrt(pos->x * pos->x + pos->y * pos->y) - dhParams[0].a;

    // 计算关节3的角度
    c3 = (r1 * r1 + pos->z * pos->z - dhParams[1].a * dhParams[1].a - dhParams[2].a * dhParams[2].a) / (2 * dhParams[1].a * dhParams[2].a);
    if (c3 > 1.0 || c3 < -1.0) {
        // 无法达到目标位置
        return 0;
    }
    s3 = sqrt(1 - c3 * c3);
    jointAngles->joint3 = atan2(s3, c3);

    // 计算关节2的角度
    c2 = (r1 * r1 + pos->z * pos->z - dhParams[1].a * dhParams[1].a - dhParams[2].a * dhParams[2].a) / (2 * dhParams[1].a * dhParams[2].a);
    if (c2 > 1.0 || c2 < -1.0) {
        // 无法达到目标位置
        return 0;
    }
    s2 = sqrt(1 - c2 * c2);
    jointAngles->joint2 = atan2(s2, c2) - atan2(dhParams[2].a * s3, dhParams[1].a + dhParams[2].a * c3);

    // 计算关节4、5、6的角度
    jointAngles->joint4 = ori->roll;
    jointAngles->joint5 = ori->pitch;
    jointAngles->joint6 = ori->yaw;

    return 1;
}


