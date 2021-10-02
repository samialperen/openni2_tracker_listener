# openni2_tracker_listener
Ros package that listens tf data published by [openni2_tracker](https://github.com/samialperen/openni2-tracker)

+ First run
```
roslaunch openni2_tracker_listener openni2_tracker_listener.launch
```
+ Wait for user to be tracked and then call following with relevant parameters (such as tracked user id, tracking enable)
```
rosservice call /openni2_tracker_listener/update_user_id "user_id: '1'
track_user: true" 
```
+ When user wants to quit or an another one came up, call the service with track_user: false
```
rosservice call /openni2_tracker_listener/update_user_id "user_id: '1'
track_user: false" 
```
