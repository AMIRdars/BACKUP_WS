import pytest

from amir_rotation_runtime_affinity import owned,role,select_groups


def test_fastest_distinct_physical_cores_keep_allowed_smt_siblings():
    rows=[(0,0,0,5000),(1,0,0,5000),(2,0,1,5200),(3,0,1,5200),
          (4,0,2,5100),(5,0,2,5100),(6,0,3,5000),(7,0,3,5000),(8,0,4,3600)]
    assert select_groups(rows,set(range(9)))==[[2,3],[4,5],[0,1],[6,7]]
    assert select_groups(rows,{0,2,4,6,8})==[[2],[4],[0],[6]]
    with pytest.raises(ValueError):select_groups(rows,{0,1,2,3})


@pytest.mark.parametrize('argv,comm,expected',[
    (['/usr/bin/python3','/ws/lib/follower_rotation_integrated'],'python3','controller'),
    (['/usr/bin/python3','/ws/lib/wrench_preprocessor'],'python3','preprocessor'),
    (['/usr/bin/python3','/ws/lib/rotation_observer'],'python3','observer'),
    (['ruby','/usr/bin/ign','gazebo','-s'],'ruby','physics'),
    (['ign gazebo'],'ign gazebo','physics'),
    (['ros2','launch','rotation_follow_integrated.launch.py'],'ros2',None)])
def test_only_actual_critical_executables_are_selected(argv,comm,expected):
    assert role(argv,comm)==expected


def test_other_domains_partitions_and_user_processes_are_never_owned():
    partition='amir_cp9_cp8r3_remeasured_case';env={b'IGN_PARTITION':partition.encode(),b'ROS_DOMAIN_ID':b'217'}
    assert owned(env,partition,'217')
    assert not owned(env,partition,'216')
    assert not owned(env,partition+'_other','217')
    assert not owned({},partition,'217')
    assert not owned({b'IGN_PARTITION':b'user',b'ROS_DOMAIN_ID':b'217'},'user','217')
