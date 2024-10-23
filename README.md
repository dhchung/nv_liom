# NV-LIOM

> [!WARNING]
> This is still a beta version of release! A lot of things are happening in my life right now. I just graduated last August, immediately got a job, and there are also some serious matters I have to deal with. My days have been busier than I expected, so I had no time to look into the code. I thought I could no longer delay and have decided to release the original code first. As you will see, the code is not organized at all! I plan to upload usage instructions and sample datasets in the future, and I will also be working on maintaining and refactoring the code little by little. Thank you so much for your interest in my work.

![E3 Mapping Result](/resources/nv_liom_ki.png)

## Description
Code repository of NV-LIOM: LiDAR-Inertial Odometry and Mapping Using Normal Vectors Towards Robust SLAM in Multifloor Environments

## Environments
- [Ubuntu 20.04](https://releases.ubuntu.com/focal/)
- [ROS1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
- [GTSAM 4.2](https://github.com/borglab/gtsam/releases/tag/4.2)

## How to use
- TBD

### Build
```
$ cd <your catkin workspace>/src
$ git clone https://github.com/dhchung/nv_liom
$ cd <your catkin workspace> && catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Mapping Results of Various Types of Buildings
### Buildings at KAIST
- [KI Building Mapping Video](https://youtu.be/Ikj619INQsE?si=pHoPqhGCv-Pb58yu) | [WebGL](http://morin.kaist.ac.kr/webgl/urls/kaist_ki.html)
- [Sports Complex Mapping Video](https://youtu.be/L8Ls0pSuess?si=268BVB-uJUzgwqLd) | [WebGL](http://morin.kaist.ac.kr/webgl/urls/sports_complex.html)
- [E3_Building Mapping Video](https://youtu.be/f1EnHy4xsRY?si=YMtSqcLuHH-CLU3r) | [WebGL](http://morin.kaist.ac.kr/webgl/urls/kaist_ee.html)
- [N1_Building Mapping Video](https://youtu.be/O1EVcwmMaPQ?si=S1yxE6PnxzGbUCjr) | [WebGL](http://morin.kaist.ac.kr/webgl/urls/kaist_n1.html)

### Public Dataset
- [SubT MRS Hawkins Multifloor](https://youtu.be/7u0Cc7jWKLM?si=XVvd7E2T7zRvBAuo)

## Resources
- Paper: [IEEE](https://ieeexplore.ieee.org/document/10670211)
- Preprint: [Arxiv](https://arxiv.org/abs/2405.12563)
- Supplementary Video: [Youtube](https://www.youtube.com/watch?v=Ln7vJHmHyeM&t=11s)

## Updates
- August 20, 2024: The paper is accepted to IEEE Robotics and Automation Letters


## Citation

```
@ARTICLE{10670211,
  author={Chung, Dongha and Kim, Jinwhan},
  journal={IEEE Robotics and Automation Letters}, 
  title={NV-LIOM: LiDAR-Inertial Odometry and Mapping Using Normal Vectors Towards Robust SLAM in Multifloor Environments}, 
  year={2024},
  volume={9},
  number={11},
  pages={9375-9382},
  doi={10.1109/LRA.2024.3457373}}
```
