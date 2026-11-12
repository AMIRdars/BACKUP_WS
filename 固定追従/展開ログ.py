"""Verify/extract the lossless evidence archives of the fixed-follow backup."""
from pathlib import Path
import argparse
import gzip
import hashlib
import json
import os
import shutil
import tempfile

ROOT=Path(__file__).resolve().parents[1]


def file_hash(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--verify',action='store_true')
    mode.add_argument('--extract',action='store_true')
    args=parser.parse_args()
    entries=json.loads((ROOT/'固定追従/圧縮ログ.json').read_text())
    for entry in entries:
        target=ROOT/entry['path'];archive=ROOT/entry['archive']
        if not target.resolve().is_relative_to(ROOT) or not archive.resolve().is_relative_to(ROOT):
            raise ValueError('archive path outside backup')
        if file_hash(archive)!=entry['archive_sha256']:raise ValueError('archive checksum: '+str(archive))
        if args.verify:
            h=hashlib.sha256()
            with gzip.open(archive,'rb') as stream:
                for block in iter(lambda:stream.read(1024*1024),b''):h.update(block)
            if h.hexdigest()!=entry['sha256']:raise ValueError('raw checksum: '+str(target))
        elif target.exists():
            if file_hash(target)!=entry['sha256']:raise ValueError('refuse to overwrite differing file: '+str(target))
        else:
            target.parent.mkdir(parents=True,exist_ok=True)
            with tempfile.NamedTemporaryFile(dir=target.parent,delete=False) as output:
                temporary=Path(output.name)
                try:
                    with gzip.open(archive,'rb') as stream:shutil.copyfileobj(stream,output,1024*1024)
                    output.flush()
                    if file_hash(temporary)!=entry['sha256']:raise ValueError('raw checksum: '+str(target))
                    if target.exists():raise FileExistsError(target)
                    os.replace(temporary,target)
                finally:temporary.unlink(missing_ok=True)
        print(('verified ' if args.verify else 'restored ')+entry['path'])
    print(str(len(entries))+' archives OK')


if __name__=='__main__':main()
