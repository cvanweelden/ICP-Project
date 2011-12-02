function D = capture_D()
    addpath('Mex')
    SAMPLE_XML_PATH='Config/SamplesConfig.xml';

    % Start the Kinect Process
    KinectHandles=mxNiCreateContext(SAMPLE_XML_PATH);

    D=mxNiDepthRealWorld(KinectHandles);

    % Stop the Kinect Process
    mxNiDeleteContext(KinectHandles);
end

 