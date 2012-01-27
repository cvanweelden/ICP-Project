function q = rot2quat(R)
%Syntax: quaternion = rot2quat(RotMatrix)
%
%Convert a given rotation matrix to quaternion.


% Author: Tahir Rabbani
% PhD student with the Section of Photogrammetry and Remote Sensing,
% Faculty of Aerospace Engineering, TU Delft, The Netherlands
% Email address: t.rabbani@lr.tudelft.nl
% Website: www.geo.tudelft.nl/frs/staff/tahir/Office/index.htm
% Sep 2004; Last revision: 15-November 2005
 

R = R(1:3,1:3);

t = 1 + trace(R);


%If the trace of the matrix is greater than zero, then
%perform an "instant" calculation.
%Important note wrt. rouning errors:

if ( t > 1e-8 )

    S = sqrt(t) * 2;
    X = (R(3,2) - R(2,3))/ S;
    Y = (R(1,3) - R(3,1))/ S;
    Z = (R(2,1) - R(1,2))/ S;
    W = 0.25 * S;


    %If the trace of the Rrix is equal to zero then identify
    %which major diagonal element has the greatest value.
    %Depending on this, calculate the following:

elseif ( R(1,1) > R(2,2) && R(1,1) > R(3,3) )      % Column 1:
    S  = sqrt( 1.0 + R(1,1) - R(2,2) - R(3,3) ) * 2;
    X = 0.25 * S;
    Y = (R(2,1) + R(1,2) ) / S;
    Z = (R(1,3) + R(3,1) ) / S;
    W = (R(3,2) - R(2,3) ) / S;
elseif ( R(2,2) > R(3,3) )             % Column 2:
    S  = sqrt( 1.0 + R(2,2) - R(1,1) - R(3,3) ) * 2;
    X = (R(2,1) + R(1,2) ) / S;
    Y = 0.25 * S;
    Z = (R(3,2) + R(2,3) ) / S;
    W = (R(1,3) - R(3,1) ) / S;
else                         % Column 3:
    S  = sqrt( 1.0 + R(3,3) - R(1,1) - R(2,2) ) * 2;
    X = (R(1,3) + R(3,1) ) / S;
    Y = (R(3,2) + R(2,3) ) / S;
    Z = 0.25 * S;
    W = (R(2,1) - R(1,2) ) / S;
end;


    %the quaternion is then defined as:
    q = [ W X Y Z ];