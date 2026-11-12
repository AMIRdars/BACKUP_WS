from af_cpu_affinity import cpu_list,select_smt


def test_selects_observed_multithread_cores_within_allowed_set(tmp_path):
    for cpu,siblings in ((0,'0-1'),(1,'0-1'),(2,'2'),(3,'3')):
        folder=tmp_path/f'cpu{cpu}/topology';folder.mkdir(parents=True)
        (folder/'thread_siblings_list').write_text(siblings)
    selected,proof=select_smt({1,2,3},tmp_path)
    assert selected=={1} and proof['1']['siblings']==[0,1]


def test_missing_topology_preserves_inherited_affinity(tmp_path):
    assert select_smt({2,3},tmp_path)[0]=={2,3}
    assert cpu_list('0-2,4,8-9')=={0,1,2,4,8,9}
