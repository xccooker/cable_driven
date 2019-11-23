#include <stdio.h>
#include <math.h>
#include "ra.h"

void test_IKinBodyNR();
int main()
{
    showTime();
    test_IKinBodyNR();
    showTime();
    return 0;
}
void test_IKinBodyNR()
{

//    showTime();
        MatrixXd Angle(7,1);
        Angle << 130, 120, 0, (34.6410/2) /180 *PI , 50, 67.720, PI/3;
//        Angle << 91.5956, 166.6118, 21.6722 /180 *PI, (34.6410/2) /180 *PI , 67.720, 67.720, (34.6410/2) /180 *PI;
        FKoutput output = ForwardKinematic(Angle);
        Matrix4d m =output.rotation * RotationZ(PI/2);
        Vector3d euler = RatationToEuler(m);
//        MatrixXd Out = InverseKinematic(Angle);
//        Matrix4d fk = fkinematic(Out);



        double start[6] = {2.875659, 622.7403, 256.2705, PI/2, PI/2, 0};
        double end[6] = {-325.33368604, 412.473077544, 412.473077544, -1.81511696409, 0.746391759961, 1.56632553046};
        Matrix4d M0; //初始位姿
        double R[3][3];
        RowVectorXd theta0(14);
        RowVectorXd theta1(14);
        ZYXEulerAngleToRot(start[3], start[4], start[5], R);
        for (int i =0; i < 14; i++)
        {
            theta0(i) = 0;
            theta1(i) = 0;
        }


        for (int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 3;j++)
                M0(i,j) = R[i][j];
        }
        M0(0,3) = start[0]; M0(1,3) = start[1]; M0(2,3) = start[2];
        M0(3,0) = 0; M0(3,1) = 0; M0(3,2) = 0; M0(3,3) = 1;
        double Ti[4][4];
        for (int i = 0; i < 4; i++)
        {
            for(int j = 0; j < 4;j++)
                Ti[i][j] = m(i,j);
        }
        InverseKinematic(theta0, theta1, M0, Ti);
        MatrixXd OUT(14,1);
        for (int i = 0; i < 14; i++)
            OUT(i) = theta1(i);
//            OUT(i) = 0;
       Matrix4d fk = fkinematic(OUT);
       PNode head = Interpolation(start, end);
       printf("start display data:");
       Dispaly(head);
        return;
}
