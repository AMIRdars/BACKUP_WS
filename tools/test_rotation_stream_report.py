"""Missing observer copies must never be silently paired to another sample."""
import json
from amir_rotation_stream_report import SequenceJoin,TimeJoin


def test_sequence_join_retains_duplicates_and_marks_a_missing_snapshot_unknown(tmp_path):
    file=tmp_path/'observations.jsonl';file.write_text(''.join(json.dumps({'publication_sequence':i,'timestamp':i*.01})+'\n' for i in [1,3,4]))
    reader=SequenceJoin(file)
    assert reader.get(1)['publication_sequence']==1
    assert reader.get(1)['publication_sequence']==1
    assert reader.get(2) is None
    assert reader.get(3)['publication_sequence']==3
    assert reader.get(4)['publication_sequence']==4
    assert reader.get(5) is None


def test_evaluation_join_uses_only_the_last_nonfuture_timestamp(tmp_path):
    file=tmp_path/'observations.jsonl';file.write_text(''.join(json.dumps({'timestamp':i*.01})+'\n' for i in [1,2,3]))
    reader=TimeJoin(file)
    assert reader.get(.005) is None
    assert reader.get(.015)['timestamp']==.01
    assert reader.get(.029)['timestamp']==.02
    assert reader.get(.04)['timestamp']==.03
