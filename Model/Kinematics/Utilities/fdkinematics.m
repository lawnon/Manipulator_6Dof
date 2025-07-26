function [Tmat,Rvec] = fdkinematics(t1, t2, t3, t4, cl, zv, path)
arguments
    t1
    t2
    t3
    t4
    cl
    zv
    path = false
end

A0_1 = ArmTrans(1, t1, t2, t3, t4)
A0_2 = ArmTrans(2, t1, t2, t3, t4)
A0_3 = ArmTrans(3, t1, t2, t3, t4)
A0_4 = ArmTrans(4, t1, t2, t3, t4)

Rot = zeros(1,3);
if (A0_4(1,1) == 0 && A0_4(2,1) == 0)
    Rot(1,1) = ToDegree(atan2(A0_4(1,2), A0_4(2,2)));
    Rot(1,2) = ToDegree(0);
    Rot(1,3) = ToDegree(0);
else
    Rot(1,1) = ToDegree(atan2(A0_4(3,2), A0_4(3,3)));
    Rot(1,2) = ToDegree(atan2(-A0_4(3,1), sqrt(A0_4(1,1)^2 + A0_4(2,1)^2)));
    Rot(1,3) = ToDegree(atan2(A0_4(2,1), A0_4(1,1)));
end

PlotArmTrans(A0_1, A0_2, A0_3, A0_4, cl, zv,path);
Tmat = A0_4;
Rvec = Rot;
end