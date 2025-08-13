function TMat = Trans2(theta, beta, alpha, a, d)
TMat = [ cos(theta)*cos(beta) -cos(alpha)*sin(theta)+sin(alpha)*sin(beta)*cos(theta)  sin(alpha)*sin(theta)+cos(alpha)*sin(beta)*cos(theta) a*cos(theta);
         sin(theta)*cos(beta)  cos(alpha)*cos(theta)+sin(alpha)*sin(beta)*sin(theta) -sin(alpha)*cos(theta)+cos(alpha)*sin(beta)*sin(theta) a*sin(theta);
        -sin(beta)             sin(alpha)*cos(beta)                                   cos(alpha)*cos(beta)                                  d           ;
         0                     0                                                      0                                                     1           ];
end