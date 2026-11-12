#!/usr/bin/env python3
"""Offline force plots / XLSX; never import ROS or modify transport evidence."""
import argparse
import hashlib
import json
import math
from pathlib import Path
from statistics import fmean

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.ticker import MultipleLocator
from openpyxl import Workbook, load_workbook
from openpyxl.chart import Reference, ScatterChart, Series
from openpyxl.drawing.image import Image
from openpyxl.styles import Alignment, Font, PatternFill
from openpyxl.utils import get_column_letter


POINTS=(('leader','left'),('leader','right'),('follower','left'),('follower','right'))
LABELS=('Leader 左指','Leader 右指','Follower 左指','Follower 右指')
COLORS=('#1F4E78','#C55A11','#548235','#7030A0')
PHASE_COLORS={'ログ前':'#EEEEEE','準備':'#D9EAF7','把持':'#FCE4D6',
    '静止保持・準備':'#E2F0D9','前進 1 m':'#DDEBF7','前進後整定':'#FFF2CC',
    '後退 1 m':'#E4DFEC','後退後整定':'#D5F0E8'}
ALPHA=.1


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def number(value):
    return float(value) if isinstance(value,(float,int)) and math.isfinite(value) else None


def sequence(row):
    phase,leg=row['phase'],row['leg']
    if phase=='motion':return '前進 1 m' if leg==1 else '後退 1 m'
    if phase=='settle' or (phase=='baseline' and leg==1):
        return '前進後整定' if leg==1 else '後退後整定'
    if phase=='closing':return '把持'
    if phase in ('startup','fixture_initialization','pre_grasp'):return '準備'
    return '静止保持・準備'


def rpy(row):
    q=[number(row.get('payload_q'+c)) for c in 'xyzw']
    if None in q:return [None]*3
    x,y,z,w=q
    return [math.degrees(math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))),
        math.degrees(math.asin(max(-1.,min(1.,2*(w*y-z*x))))),
        math.degrees(math.atan2(2*(w*z+x*y),1-2*(y*y+z*z)))]


def label_style(ax):
    ax.grid(axis='y',color='#CCD3DA',linewidth=.6)
    ax.spines[['top','right']].set_visible(False)
    ax.tick_params(labelsize=10)


def bands(ax,segments):
    for start,end,label in segments:
        ax.axvspan(start,end,color=PHASE_COLORS[label],alpha=.38,zorder=0)


def timeline(ax,segments,start,end):
    for left,right,label in segments:
        a,b=max(left,start),min(right,end)
        if b<=a:continue
        ax.broken_barh([(a,b-a)],(.1,.8),facecolors=PHASE_COLORS[label],edgecolors='#8B9299',linewidth=.6)
        text=label.replace('・','・\n').replace(' 1 m','\n1 m').replace('後整定','後\n整定')
        if b-a>=4:ax.text((a+b)/2,.5,text,ha='center',va='center',fontsize=9)
    ax.set_ylim(0,1);ax.set_yticks([]);ax.set_xlim(start,end)
    ax.set_xlabel('シミュレーション時刻 [s]（/clock）',fontsize=12)
    ax.xaxis.set_major_locator(MultipleLocator(10 if end-start<100 else 20))
    ax.spines[['left','right','top']].set_visible(False)


def force_figure(data,segments,indices,title):
    fig,axes=plt.subplots(2,1,figsize=(15,6.8),sharex=True,
        gridspec_kw={'height_ratios':[5.0,.8]},constrained_layout=True)
    ax,bar=axes;bands(ax,segments)
    t=[r[0] for r in data]
    for i in indices:
        ax.plot(t,[r[5+i] for r in data],color=COLORS[i],alpha=.22,linewidth=.7)
        ax.plot(t,[r[9+i] for r in data],color=COLORS[i],linewidth=1.5,label=LABELS[i])
    ax.set_ylim(0,25);ax.yaxis.set_major_locator(MultipleLocator(5))
    ax.set_ylabel('指センサ法線力 |Fx| [N]',fontsize=12)
    ax.set_title(title,fontsize=18,loc='left',pad=16)
    ax.legend(loc='upper left',ncol=2 if len(indices)==4 else 1,frameon=True,fontsize=11)
    label_style(ax);ax.tick_params(labelbottom=False)
    end=math.ceil(t[-1]/10)*10;timeline(bar,segments,0,end)
    fig.supxlabel('淡色線：記録値　実線：表示用の指数平滑（α=0.1/記録サンプル）。欠測は補間していません。',fontsize=10,color='#46515C')
    return fig


def transport_figure(data,segments):
    start=min(r[0] for r in data if r[3]=='motion')
    selected=[r for r in data if r[0]>=start]
    t=[r[0] for r in selected]
    fig,axes=plt.subplots(5,1,figsize=(15,12),sharex=True,
        gridspec_kw={'height_ratios':[2.,2.,2.,2.,.7]},constrained_layout=True)
    for ax in axes[:-1]:bands(ax,segments);label_style(ax);ax.tick_params(labelbottom=False)
    for i in range(4):axes[0].plot(t,[r[13+i] for r in selected],color=COLORS[i],linewidth=1.,label=LABELS[i])
    axes[0].set_ylabel('指力変化 ΔF [N]');axes[0].legend(ncol=4,loc='upper right',fontsize=9)
    axes[0].set_title('搬送区間の力と変位｜前進・停止・後退・停止',loc='left',fontsize=18,pad=16)
    for col,color,label,style in ((17,COLORS[0],'Leader Fx','-'),(20,COLORS[3],'Follower Fx','--')):
        axes[1].plot(t,[r[col] for r in selected],color=color,label=label,linestyle=style,linewidth=1.2)
    axes[1].set_ylabel('手首F/T由来 Fx [N]');axes[1].legend(ncol=2,fontsize=9)
    for col,color,label in ((29,COLORS[3],'Follower filtered Fx'),(30,COLORS[2],'Follower filtered Fy')):
        axes[2].plot(t,[r[col] for r in selected],color=color,label=label,linewidth=1.2)
    axes[2].set_ylabel('フォロワ制御入力 [N]');axes[2].legend(ncol=2,fontsize=9)
    axes[3].plot(t,[r[34] for r in selected],color=COLORS[0],linewidth=1.6,label='搬送物 ΔX（独立GT）')
    axes[3].set_ylabel('搬送前基準の変位 [m]');axes[3].legend(loc='upper right',fontsize=9)
    timeline(axes[-1],segments,start,t[-1])
    fig.supxlabel('ΔF・ΔXの基準：搬送前の静止保持末尾1 sim秒の平均。手首F/Tは各台の自機座標・把持点周り（補償前）。',fontsize=10,color='#46515C')
    return fig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,required=True)
    parser.add_argument('--reference',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source,reference,out=(p.expanduser().resolve() for p in (args.source,args.reference,args.output))
    if out.exists():raise ValueError('preserve existing output; select a new directory: '+str(out))
    inputs=[source/n for n in ('samples.jsonl','timeseries.csv','result.json')]+[reference]
    before={str(p):sha(p) for p in inputs}
    # Reference workbook is inspected only; data below comes from this run.
    ref=load_workbook(reference,read_only=True,data_only=True)
    reference_sheets=ref.sheetnames;ref.close()
    rows=[json.loads(line) for line in (source/'samples.jsonl').open()]
    assert rows and all(rows[i]['sim_time']<=rows[i+1]['sim_time'] for i in range(len(rows)-1))
    hold=[r for r in rows if r['phase']=='static_hold']
    baseline_rows=[r for r in hold if r['sim_time']>=hold[-1]['sim_time']-1.]
    def forces(r):
        return [number(r.get('native_grasp_monitor',{}).get(robot,{}).get('own_forces_N',{}).get(side)) for robot,side in POINTS]
    baseline=[fmean(forces(r)[i] for r in baseline_rows if forces(r)[i] is not None) for i in range(4)]
    origin=[fmean(r['payload_'+c] for r in baseline_rows) for c in 'xyz']
    rotation_origin=[fmean(rpy(r)[i] for r in baseline_rows) for i in range(3)]
    smooth=[None]*4;data=[]
    for r in rows:
        raw=forces(r)
        display=[]
        for i,value in enumerate(raw):
            if value is None:smooth[i]=None;display.append(None)
            else:
                smooth[i]=value if smooth[i] is None else ALPHA*value+(1-ALPHA)*smooth[i]
                display.append(smooth[i])
        delta=[None if x is None else x-baseline[i] for i,x in enumerate(raw)]
        pose=[number(r.get('payload_'+c)) for c in 'xyz'];rotation=rpy(r)
        pose_delta=[None if x is None else x-origin[i] for i,x in enumerate(pose)]
        angle_delta=[None if x is None else (x-rotation_origin[i]+180)%360-180 for i,x in enumerate(rotation)]
        wrist=[number(r.get(robot+'_'+c)) for robot in ('leader','follower') for c in ('fx','fy','fz')]
        torques=[number(r.get(robot+'_'+c)) for robot in ('leader','follower') for c in ('tx','ty','tz')]
        filtered=[number(r.get('filtered_f'+c+'_N')) if r.get('admittance_enabled') else None for c in 'xy']
        monitors=r.get('native_grasp_monitor',{})
        data.append([r['sim_time'],r['sim_time']-rows[0]['sim_time'],sequence(r),r['phase'],r['leg']]
            +raw+display+delta+wrist+torques+filtered+pose+pose_delta+rotation+angle_delta
            +[number(r.get(robot+'_encoder_vx')) for robot in ('leader','follower')]
            +[monitors.get(robot,{}).get('healthy') for robot in ('leader','follower')]
            +[number(monitors.get(robot,{}).get('sim_time')) for robot in ('leader','follower')]
            +[r['wall_time']])
    segments=[]
    if data[0][0]>0:segments.append((0,data[0][0],'ログ前'))
    start,label=data[0][0],data[0][2]
    for row in data[1:]:
        if row[2]!=label:segments.append((start,row[0],label));start,label=row[0],row[2]
    segments.append((start,data[-1][0],label))
    out.mkdir(parents=True)
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    font_manager.fontManager.addfont(str(font))
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=str(font)).get_name(),
        'axes.unicode_minus':False,'pdf.fonttype':3,'savefig.facecolor':'white'})
    figures=[('把持力_4指とシーケンス.png',force_figure(data,segments,range(4),'1 m前進・1 m後退｜把持力とシーケンス（4指）')),
        ('右指_把持力とシーケンス.png',force_figure(data,segments,(1,3),'1 m前進・1 m後退｜両台の右指把持力')),
        ('搬送区間_力と変位.png',transport_figure(data,segments))]
    with PdfPages(out/'力推移グラフ.pdf') as pdf:
        for name,fig in figures:
            fig.savefig(out/name,dpi=150);pdf.savefig(fig);plt.close(fig)

    book=Workbook();summary=book.active;summary.title='概要'
    sheet=book.create_sheet('時系列データ');right=book.create_sheet('右指グラフ')
    ft=book.create_sheet('手首FTと搬送');charts=book.create_sheet('編集用グラフ')
    stats=book.create_sheet('区間と統計');notes=book.create_sheet('説明')
    headers=['時刻 sim_time [s]','ログ開始から [s]','シーケンス','phase','leg']
    headers += [label+' 法線力 |Fx| [N]' for label in LABELS]
    headers += [label+' 表示用EMA [N]' for label in LABELS]
    headers += [label+' 静止基準からの変化 [N]' for label in LABELS]
    headers += [robot+' 手首F/T由来 '+c+' [N]' for robot in ('Leader','Follower') for c in ('Fx','Fy','Fz')]
    headers += [robot+' 手首F/T由来 '+c+' [Nm]' for robot in ('Leader','Follower') for c in ('Tx','Ty','Tz')]
    headers += ['Follower filtered Fx [N]','Follower filtered Fy [N]']
    headers += ['搬送物 '+c+' [m]' for c in 'XYZ']+['搬送物 静止基準Δ'+c+' [m]' for c in 'XYZ']
    headers += ['搬送物 '+c+' [deg]' for c in ('Roll','Pitch','Yaw')]+['搬送物 基準Δ'+c+' [deg]' for c in ('Roll','Pitch','Yaw')]
    headers += ['Leader 自機encoder Vx [m/s]','Follower 自機encoder Vx [m/s]',
        'Leader 把持監視healthy','Follower 把持監視healthy','Leader 把持監視時刻 [s]',
        'Follower 把持監視時刻 [s]','wall_time（monotonic）[s]']
    assert len(headers)==len(data[0])==50
    sheet.append(headers)
    for row in data:sheet.append(row)
    sheet.freeze_panes='F2';sheet.auto_filter.ref=sheet.dimensions
    for col in sheet.columns:
        index=col[0].column;sheet.column_dimensions[get_column_letter(index)].width=22 if index>5 else (23 if index==3 else 18)
        for cell in col[1:]:
            if isinstance(cell.value,(float,int)) and not isinstance(cell.value,bool):cell.number_format='0.000000'
    for row in sheet.iter_rows(min_row=2,max_col=5):
        fill=PatternFill('solid',fgColor=PHASE_COLORS[row[2].value].lstrip('#'))
        for cell in row:cell.fill=fill
    result=json.loads((source/'result.json').read_text())
    summary_rows=[['1 m前進・1 m後退｜力の推移',''],['項目','値'],
        ['入力試験',source.name],['試験結果','PASS' if result.get('passed') else 'FAIL'],
        ['記録点数',len(data)],['先頭sim_time [s]',data[0][0]],['末尾sim_time [s]',data[-1][0]],
        ['記録期間 [sim s]',data[-1][0]-data[0][0]],['前進GT物体移動 [m]',result['legs'][0]['bodies']['cooperative_payload']['signed_displacement_m']],
        ['後退GT物体移動 [m]',result['legs'][1]['bodies']['cooperative_payload']['signed_displacement_m']],
        ['往復後の物体X残差 [mm]',result['round_trip_net_gt_xy_yaw']['cooperative_payload'][0]*1000],
        ['平滑化','EMA α=0.1 / 記録サンプル（表示用のみ）'],
        ['基準窓 [sim s]',f"{baseline_rows[0]['sim_time']:.3f}–{baseline_rows[-1]['sim_time']:.3f}"],
        ['注記','4指の力はnative_grasp_monitor.own_forces_N（各指センサの|Fx|）。'],
        ['座標','手首F/Tは各台基準・把持点周り。filtered列と区別。'],
        ['参照Excel',reference.name],['データ欠測','空欄を保持。ゼロ埋め・補間なし。']]
    for row in summary_rows:summary.append(row)
    def preview(ws,path,anchor,width=1200):
        img=Image(str(path));img.height*=width/img.width;img.width=width;ws.add_image(img,anchor)
    preview(summary,out/figures[0][0],'A20');preview(right,out/figures[1][0],'A1')
    preview(ft,out/figures[2][0],'A1')
    stats.append(['区間','開始sim_time [s]','終了sim_time [s]','期間 [s]'])
    for a,b,label in segments:stats.append([label,a,b,b-a])
    stats.append([]);stats.append(['区間','把持点','有効点数','最小 [N]','平均 [N]','最大 [N]','静止基準 [N]'])
    for name,select in (('全記録',lambda x:True),('前進',lambda x:x[3]=='motion' and x[4]==1),('後退',lambda x:x[3]=='motion' and x[4]==2)):
        for i,label in enumerate(LABELS):
            values=[x[5+i] for x in data if select(x) and x[5+i] is not None]
            stats.append([name,label,len(values),min(values),fmean(values),max(values),baseline[i]])
    notes_rows=[['項目','内容'],['入力samples.jsonl',str(source/'samples.jsonl')],
        ['参照Excel',str(reference)],['参照シート',' / '.join(reference_sheets)],
        ['データの区別','参照Excelは様式のみ。数値はpublic_gui_01のログから抽出。'],
        ['生値列','元の監視ログの指力をそのまま転記。手首Fxで代用しない。'],
        ['平滑値列','y[n]=0.1*x[n]+0.9*y[n-1]。元の値は別列に保持。'],
        ['指力の基準','static_hold末尾1 sim秒の各指平均。制御のtare操作ではない。'],
        ['補正変化','記録値−静止基準の符号付き変化。絶対値を取り直していない。'],
        ['時刻','/clock由来sim_timeを数値X軸として使う。壁時計の秒と区別。'],
        ['区間','phaseとlegの記録上の切替。時刻補間・データ間引きなし。'],
        ['欠測','初期の空欄・非有限値は空欄。未有効時のfiltered列は空欄。'],
        ['手首F/T','センサ値を各台基準座標・把持点周りへ変換した補償前値。'],
        ['対向配置','各台の自機座標の符号を世界座標の符号と混同しない。'],
        ['表示','両台の曲線が近いと重なって見える。'],
        ['記録周期','元ハーネスは約0.05 wall秒ごとに最新値を記録。不等間隔sim_time。'],
        ['独立GT','物体位置と姿勢は評価専用。制御へ入力しない。']]
    for path,digest in before.items():notes_rows.append(['SHA256 '+Path(path).name,digest])
    for row in notes_rows:notes.append(row)
    def add_chart(title,cols,anchor,unit,xcol=1):
        chart=ScatterChart();chart.title=title;chart.style=13;chart.width=29;chart.height=12
        chart.x_axis.title='sim_time [s]' if xcol==1 else headers[xcol-1]
        chart.y_axis.title=unit;chart.scatterStyle='line';chart.display_blanks='gap'
        x=Reference(sheet,min_col=xcol,min_row=2,max_row=sheet.max_row)
        for i,col in enumerate(cols):
            series=Series(Reference(sheet,min_col=col,min_row=1,max_row=sheet.max_row),xvalues=x,title_from_data=True)
            color_index=(col-6)%4 if 6<=col<=17 else (3 if col==21 else i%4)
            series.graphicalProperties.line.solidFill=COLORS[color_index].lstrip('#');series.graphicalProperties.line.width=18000
            chart.series.append(series)
        if xcol==1:chart.x_axis.scaling.min=0;chart.x_axis.majorUnit=20
        charts.add_chart(chart,anchor)
    for params in [('4指の法線力（表示用EMA）',list(range(10,14)),'A1','法線力 [N]'),
        ('両台の右指（表示用EMA）',[11,13],'A25','法線力 [N]'),
        ('搬送前静止基準からの指力変化',list(range(14,18)),'A49','符号付き変化 [N]'),
        ('手首F/T由来 Fx（各台の自機座標）',[18,21],'A73','力 [N]'),
        ('搬送物 ΔX（独立GT）',[35],'A97','静止基準変位 [m]'),
        ('搬送物の姿勢変化（独立GT）',[41,42,43],'A121','姿勢変化 [deg]')]:add_chart(*params)
    add_chart('搬送物のX–Y軌跡（独立GT）',[33],'A145','Y [m]',xcol=32)
    for ws in book:
        ws.sheet_view.showGridLines=False
        if ws!=sheet:
            for col in range(1,8):ws.column_dimensions[get_column_letter(col)].width=26
        ws.row_dimensions[1].height=34
        for cell in ws[1]:cell.fill=PatternFill('solid',fgColor='1F4E78');cell.font=Font(color='FFFFFF',bold=True);cell.alignment=Alignment(wrap_text=True,vertical='center')
    summary.column_dimensions['A'].width=36;summary.column_dimensions['B'].width=90
    notes.column_dimensions['A'].width=32;notes.column_dimensions['B'].width=125
    for ws in (summary,notes):
        for row in ws:
            for cell in row:cell.alignment=Alignment(wrap_text=True,vertical='top')
            ws.row_dimensions[row[0].row].height=30
    path=out/'往復搬送_力推移.xlsx';book.save(path)
    check=load_workbook(path,read_only=False,data_only=True)
    assert check['時系列データ'].max_row==len(data)+1
    assert len(check['編集用グラフ']._charts)==7
    for i in (0,len(data)//2,len(data)-1):
        for col in (1,6,7,8,9,18,21,35):
            actual=check['時系列データ'].cell(i+2,col).value;expected=data[i][col-1]
            assert actual is None and expected is None or math.isclose(actual,expected,rel_tol=1e-12,abs_tol=1e-12)
    check.close()
    assert all(sha(p)==digest for p,digest in before.items()),'input unexpectedly changed'
    manifest={'source':str(source),'reference':str(reference),'input_sha256':before,'samples':len(data),
        'sim_time_range_s':[data[0][0],data[-1][0]],'baseline_window_s':[baseline_rows[0]['sim_time'],baseline_rows[-1]['sim_time']],
        'baseline_normal_forces_N':dict(zip(LABELS,baseline)),
        'outputs':{p.name:sha(p) for p in out.iterdir() if p.is_file()},
        'validation':{'input_hashes_unchanged':True,'xlsx_row_count_correct':True,'editable_charts':7,
            'selected_source_values_read_back':True,'no_interpolation_or_decimation':True}}
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    (out/'README.md').write_text('# 往復搬送の力推移\n\n入力はpublic_gui_01のsamples.jsonl。直進.xlsxはグラフの様式のみ参照。\n\n'
        '往復搬送_力推移.xlsxには生の記録値、表示用EMA、静止基準との差、手首F/T、評価用GTと7つの編集可能グラフを保存。'
        'PNG3枚とPDF3ページは同じデータから生成。時刻はsim_timeで、欠測をゼロ埋めせず、補間・間引きなし。'
        '詳細な項目定義はExcelの説明シート、元入力と出力のSHA256はmanifest.jsonを参照。\n')
    print(json.dumps({'output':str(out),'samples':len(data),'files':[p.name for p in out.iterdir()]},ensure_ascii=False,indent=2))


if __name__=='__main__':main()
