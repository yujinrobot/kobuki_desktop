^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kobuki_dashboard
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix compilation on OS X.
  - without this linking fails on OS X (10.9)
  - prefer find_package over pkg-config, since pkg-config does not link
  transitive dependencies (see https://bitbucket.org/osrf/gazebo/issue/1202)
* Contributors: Nikolaus Demmel

0.3.2 (2014-04-23)
------------------
* removes email addresses from authors
* replace deprecated shared_dynamic_cast (fixes `#25 <https://github.com/yujinrobot/kobuki_desktop/issues/25>`_)
* Contributors: Marcus Liebhardt, Samir Benmendil

0.3.1 (2013-10-14)
------------------
* fixes gazebo header paths (refs `#22 <https://github.com/yujinrobot/kobuki_desktop/issues/22>`_)

0.3.0 (2013-08-30)
------------------
* fixes bumper & cliff event publishing
* adds reset odometry and fixes imu and odom data processing
* adds IMU data processing
* adds bugtracker and repo info to package.xml

0.2.0 (2013-07-11)
------------------
* ROS Hydro beta release.
* Adds catkinized kobuki_qtestsuite
