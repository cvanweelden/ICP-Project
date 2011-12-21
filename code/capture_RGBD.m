function [I,D] = capture_RGBD()

    CONFIG_XML_PATH='KinectConfig.xml';

    % Start the Kinect Process
    KinectHandles=mxNiCreateContext(CONFIG_XML_PATH);

    I=mxNiPhoto(KinectHandles); I=permute(I,[3 2 1]);
    D=mxNiDepthRealWorld(KinectHandles);

    % Stop the Kinect Process
    mxNiDeleteContext(KinectHandles);
end

 