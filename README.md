The goal of this project was to register depth images captured using a Kinect to create a global point cloud model. You can see some results on our [log](https://github.com/cvanweelden/ICP-Project/wiki/ResearchLog). Also see our [final report](https://github.com/downloads/cvanweelden/ICP-Project/report.pdf).

# Registration code using ICP

In the cpp folder there are several procedures for registering point clouds and rgb-d datasets and for evaluation of those procedures. All of this depends on the [Point Cloud Library (PCL)](http://pointclouds.org/) of which we used version 1.4.0. 

## Quickstart

You will need cmake (we used version 2.8.7) and c++ compilers (we used those that come with version 3.2.6 of Xcode). First clone the repo and cd to the created directory, then issue the following commands.

```bash
$cd cpp
$mkdir build
$cd build
$cmake ../
$make
```

This will create several executables in your build folder, notably `register` and `evaluation`. For more details see the [Wiki](https://github.com/cvanweelden/ICP-Project/wiki)