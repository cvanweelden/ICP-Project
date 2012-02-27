function D = capture_D()

    CONFIG_XML_PATH='KinectConfig.xml';

    % Start the Kinect Process
    KinectHandles=mxNiCreateContext(CONFIG_XML_PATH);

    D=mxNiDepthRealWorld(KinectHandles);

    % Stop the Kinect Process
    mxNiDeleteContext(KinectHandles);
end

 