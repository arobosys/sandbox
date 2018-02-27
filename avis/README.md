# Репо по пакету AVIS

## Запуск программы в логическом уровне
* Я перенес файл kernel.launch от /home/zhuhua/AGRO/avis2/kernel/launch 
* до /home/zhuhua/AGRO/avis2/kernel/src/logic_layer/launch
```bash
 cd AGRO/avis2/kernel/
 source devel/setup.bash 
 export AVIS_ROOT=/home/zhuhua/AGRO/avis2
 roslaunch logic_layer kernel.launch
```


## Опубликовать топик /CI_INput
```bash
rostopic pub  /CI_input std_msgs/String "start Exhibition 7 5"
```


## Информации на экране
```bash
[ INFO] [1519721394.635069591]: scenario_executor_server started!
[ INFO] [1519721394.644173179]: TechmapServer started
[ INFO] [1519721394.646625787]: RosLinkServer started
[ INFO] [1519721402.091109834]: CI successfuly parsed a command
[ INFO] [1519721402.291996125]: Got a goal: start Exhibition
[ INFO] [1519721402.292021808]: Alias = Exhibition
[ INFO] [1519721402.295235748]: Got scenario from DB
[ INFO] [1519721402.306891156]: TechmapClient waiting for server
[ INFO] [1519721402.593103932]: TechmapClient sending goal (ID=3)
[ INFO] [1519721402.593399478]: TechmapClient waiting for result
[ INFO] [1519721402.593746059]: TechmapServer executing tmap (ID=3)
[ INFO] [1519721402.593772557]: ID = 3
[ INFO] [1519721402.609893158]: TaskListExecClient waiting for server
[ INFO] [1519721402.911957199]: Started process server with name <motionctl_parent>
[ INFO] [1519721402.914906596]: Started process server with name <navigation_parent>
[ INFO] [1519721402.931816420]: Started process server with name <trajectory_calculator_node>
[ INFO] [1519721402.977393905]: Waiting for valid clock time...
[ INFO] [1519721402.977428266]: Valid clock time received. Starting node.
[ INFO] [1519721403.395223317]: TechMapExecClient tmap execution ret state: OK
[ INFO] [1519721403.395710956]: [CI feedback : stateHLL] 
[ INFO] [1519721403.395749825]: [CI feedback : stateLLL] 
[ INFO] [1519721403.403097633]: TaskListExecClient waiting for server
[ INFO] [1519721403.458977999]: Started process server with name <algo>
[ INFO] [1519721403.466707721]: Started process server with name <steering_delta_calculator>
[ INFO] [1519721403.476793614]: AlgoXCorr started
[ INFO] [1519721403.942264765]: Child process started: <algo>
[ INFO] [1519721403.942265259]: Child process started: <steering_delta_calculator>
[ INFO] [1519721404.205711515]: Child process connected: <steering_delta_calculator>
[ INFO] [1519721404.206488218]: Child process connected: <algo>
[ INFO] [1519721404.207854436]: received options from parent.
[ INFO] [1519721404.207920683]: `start` handler working..
`START` HANDLER WORKING
[ INFO] [1519721404.224149688]: Started process server with name <steering_reverse_kinematic_calc>
[ INFO] [1519721404.706822007]: Child process started: <steering_reverse_kinematic_calc>
[ INFO] [1519721405.007786999]: Child process connected: <steering_reverse_kinematic_calc>
[ INFO] [1519721405.027298354]: Started process server with name <steering_speed_calc>
[ INFO] [1519721405.508644388]: Child process started: <steering_speed_calc>
[ INFO] [1519721405.810514269]: Child process connected: <steering_speed_calc>
TurnTrajCalculator error: "tf_map" passed to lookupTransform argument target_frame does not exist. 
. Please retry. Transforms from goal will be applied[ INFO] [1519721408.449164727]: Trajectory published
[ INFO] [1519721408.455874803]: Got goal with delta = 1.000000
```




