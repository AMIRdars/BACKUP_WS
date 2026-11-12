"""Measurement semantics: silence/replay/header loss must not imply qualification."""
import importlib.util
import json
from pathlib import Path
import pytest

spec=importlib.util.spec_from_file_location('observer',Path(__file__).with_name('amir_cp9_observe.py'))
observer=importlib.util.module_from_spec(spec);spec.loader.exec_module(observer)


def row(received,stamp,finite=True):
    return {'receive_monotonic_s':received,'source_stamp_s':stamp,'finite_payload':finite}


def test_silence_has_no_rate_or_qualification():
    r=observer.summarize([],10)
    assert r['arrival_rate_hz'] is None and r['observation_status']=='NO_SAMPLES'
    assert not r['hardware_qualified'] and not r['firmware_timeout_measured']


def test_gaps_and_stale_republication_are_distinct():
    r=observer.summarize([row(1,10),row(1.01,10),row(1.02,10),row(1.1,9),row(1.11,9.01,False)],1)
    assert r['arrival_rate_hz']==pytest.approx(4/.11)
    assert r['duplicate_source_stamps']==2 and r['regressing_source_stamps']==1
    assert r['arrival_gap_max_s']==pytest.approx(.08)
    assert r['nonfinite_payload_samples']==1
    assert not r['end_to_end_actuator_latency_measured'] and not r['hardware_qualified']


def test_headerless_messages_do_not_invent_source_period():
    r=observer.summarize([row(1,None),row(1.01,None)],1)
    assert r['arrival_rate_hz']==pytest.approx(100)
    assert r['header_samples']==0 and r['source_period_median_s'] is None


def test_single_and_zero_stamp():
    r=observer.summarize([row(1,0)],1)
    assert r['arrival_rate_hz'] is None and r['zero_source_stamps']==1


@pytest.mark.parametrize('value,expected',[(None,None),({},None),({'header':{'stamp':{'sec':2,'nanosec':500000000}}},2.5),({'header':{'stamp':{'sec':2,'nanosec':1000000000}}},None)])
def test_stamp(value,expected):
    assert observer.message_stamp(value or {})==expected


@pytest.mark.parametrize('value,expected',[({'a':[1.,2.]},True),({'a':[float('nan')]},False),({'a':{'b':float('inf')}},False),({'a':[False,'unknown',None]},True)])
def test_nested_nonfinite(value,expected):
    assert observer.finite_payload(value)==expected


@pytest.mark.parametrize('duration',[0,-1,float('inf'),float('nan')])
def test_bad_window(duration):
    with pytest.raises(ValueError):observer.summarize([],duration)


@pytest.mark.parametrize('topics',[[],[{'topic':'relative','type':'std_msgs/msg/Bool'}],[{'topic':'/a','type':'Bool'}],[{'topic':'/a','type':'std_msgs/msg/Bool'}]*2])
def test_bad_topic_spec(tmp_path,topics):
    p=tmp_path/'topics.json';p.write_text(json.dumps({'topics':topics}))
    with pytest.raises(ValueError):observer.load_topics(p)
