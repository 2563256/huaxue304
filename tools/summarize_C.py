#!/usr/bin/python3
"""Create a measured full-batch report and a task timeline from an audit pass."""
import argparse
import csv
import json
from pathlib import Path
import statistics


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();audit=json.loads(args.audit.read_text())
    assert audit['passed']
    data=audit['metrics'];summary=data['summary'];rows=sorted(data['tasks'],key=lambda r:r['task_id'])
    assert summary['tasks_completed']==summary['tasks_submitted']==10 and summary['tasks_failed']==0
    assert len(rows)==len({r['task_id'] for r in rows})==10
    assert all(r['completed_at_sec'] not in ('',None) for r in rows)
    mode=audit['mapf']['mode'].upper()
    origin=min(r['created_at_sec'] for r in rows)
    assert abs(max(r['completed_at_sec'] for r in rows)-origin-summary['makespan_sec'])<.005
    waits=[r['assigned_at_sec']-r['created_at_sec'] for r in rows]
    execution=[r['completed_at_sec']-r['assigned_at_sec'] for r in rows]
    additional=dict(mean_queue_wait_sec=round(statistics.mean(waits),3),
        mean_execution_sec=round(statistics.mean(execution),3),
        task_distance_m=round(sum(r['distance_m'] for r in rows),3),
        task_energy_wh=round(sum(r['energy_wh'] for r in rows),4),
        task_requeues=sum(r['requeue_count'] for r in rows),
        wall_to_report_sec=audit['wall_to_report_sec'])
    out=args.output;out.mkdir(parents=True,exist_ok=True)
    (out/'十任务完整验证结果.json').write_text(json.dumps(dict(audit=audit,derived=additional),ensure_ascii=False,indent=2))
    with (out/'十任务逐项指标.csv').open('w',newline='',encoding='utf-8-sig') as f:
        fields=['task_id','robot_id','payload_kg','created_at_sec','assigned_at_sec','completed_at_sec',
            'queue_wait_sec','execution_sec','completion_time_sec','distance_m','energy_wh','requeue_count','tardy']
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader()
        for row,wait,run in zip(rows,waits,execution):w.writerow(dict(row,queue_wait_sec=round(wait,3),execution_sec=round(run,3)))
    metrics=[('提交／完成／失败',f"10／10／0"),('完成率','100%'),
        ('Makespan',f"{summary['makespan_sec']:.3f} 秒"),
        ('平均任务完成时间（提交至完成）',f"{summary['mean_completion_time_sec']:.3f} 秒"),
        ('平均排队等待（提交至派单）',f"{additional['mean_queue_wait_sec']:.3f} 秒"),
        ('平均执行时间（派单至完成，含交通等待与装卸）',f"{additional['mean_execution_sec']:.3f} 秒"),
        ('调度器统计总里程',f"{summary['total_distance_m']:.3f} 米"),
        ('模型估算总能耗',f"{summary['total_energy_wh']:.3f} Wh"),
        ('车队平均任务利用率',f"{summary['utilisation']*100:.2f}%"),
        ('重排次数',str(summary['reschedule_count'])),('任务重新入队次数',str(additional['task_requeues'])),
        ('超期任务',str(summary['tardy_tasks'])),('验证墙钟耗时（提交至指标落盘）',f"{audit['wall_to_report_sec']:.1f} 秒")]
    lines=[f'# 副本 C / {mode}：十任务连续仿真验证与指标','',
        '本报告来自修复后从初始车位启动的一次独立 Gazebo 批次。十项任务保持原 ID、任务参数与提交顺序，未中途重启调度器、手动移车、重新命名补跑或合并旧批次。',
        '', '场景：`src/hetero_fleet_scheduler/config/tasks/demo_10.yaml`。所有四套 Nav2 的规划器、控制器、导航器生命周期激活后，使用原任务发布器按 `--interval 8` 提交。',
        '', '## 汇总','', '| 指标 | 结果 |','|---|---|']
    lines += [f'| {name} | {value} |' for name,value in metrics]
    lines += ['', '除墙钟耗时外，时间指标均采用仿真时间。Makespan 从本批次首项任务提交算到最后一项完成；平均完成时间包含排队。总里程与能耗为项目调度器的统计及估算，能耗来自车型、载荷与里程模型，不是电表测量。空驶和已计入的让位/返航会影响车队总量；最后完成后的返航不计入 Makespan。',
        '', '利用率为四台机器人的任务忙碌时间之和除以 `4 × Makespan`，不把让位、返航算作任务忙碌时间；任务内的交通等待和装卸仍计入忙碌时间，因此这里不是纯运动利用率。单次成功运行证明该场景能够执行完成，不构成最优 Makespan 或相对旧方法提升的证明。',
        '', '## 每台机器人','', '| 机器人 | 完成任务 | 统计里程 m | 估算能耗 Wh | 任务忙碌 s | 电量 % |','|---|---:|---:|---:|---:|---:|']
    for name,r in summary['per_robot'].items():
        lines.append(f"| {name} | {r['tasks_completed']} | {r['distance_m']:.3f} | {r['energy_wh']:.3f} | {r['busy_sec']:.3f} | {r['battery_pct']:.1f} |")
    lines += ['', '## 十项任务','', '| 任务 | 机器人 | 等待 s | 执行 s | 提交至完成 s | 统计里程 m | 估算能耗 Wh |','|---|---|---:|---:|---:|---:|---:|']
    for r,wait,run in zip(rows,waits,execution):
        lines.append(f"| {r['task_id']} | {r['robot_id']} | {wait:.3f} | {run:.3f} | {r['completion_time_sec']:.3f} | {r['distance_m']:.3f} | {r['energy_wh']:.3f} |")
    lines += ['', '## 方法运行证据', '',
        f"本次实际模式：{mode}；原生路线协调请求 {audit['mapf']['requests']} 次；临时拒绝 {audit['mapf']['rejected']} 次；选择绕行候选 {audit['mapf']['alternative_routes_selected']} 次。临时拒绝会停止当前规划并重试，不等同于任务失败。", '',
        'CBS/EECBS 只对有限车身可行候选目录及离散时间模型给出代价性质；不构成全连续空间最优解、实际 Makespan 次优界限或相对 A 提升的证明。当前连续瓶颈仍需独占准入。', '',
        f"实际平面物理车身对静态地图的逐位姿检查：碰撞样本 {audit['physical_body_audit']['collision_samples']}，各机器人约 {min(audit['physical_body_audit']['pose_samples'].values())} 个采样。该检查不构成未测量的动态碰撞统计。", '',
        '构建、373 项 Python 检查、9 项原生检查、原文件保护与同批次代码校验结果见副本 C 验证记录。', '',
        '![任务时间线](十任务时间线.png)', '']
    (out/'十任务完整验证与指标.md').write_text('\n'.join(lines))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={'light_1':'#2563eb','heavy_1':'#dc2626','omni_1':'#059669','g1_1':'#9333ea'}
    fig,ax=plt.subplots(figsize=(12,6.2),layout='constrained')
    for i,(r,wait,run) in enumerate(zip(rows,waits,execution)):
        ax.barh(i,wait,left=r['created_at_sec']-origin,height=.22,color='#d1d5db')
        ax.barh(i,run,left=r['assigned_at_sec']-origin,height=.62,color=colors[r['robot_id']])
    from matplotlib.patches import Patch
    ax.set_yticks(range(len(rows)),[r['task_id'].replace('balanced_10_','Task ')+' / '+r['robot_id'] for r in rows]);ax.invert_yaxis()
    ax.set_xlabel('Simulation seconds since first task submission');ax.set_title(f"{mode} | 10/10 completed, 0 failed | Makespan {summary['makespan_sec']:.3f} s")
    ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    ax.legend(handles=[Patch(color='#d1d5db',label='Queue wait')]+[Patch(color=c,label=n) for n,c in colors.items()],loc='lower right',fontsize=8)
    fig.savefig(out/'十任务时间线.png',dpi=160);fig.savefig(out/'十任务时间线.svg');plt.close(fig)
    print(json.dumps(dict(summary=summary,derived=additional),ensure_ascii=False,indent=2))


if __name__=='__main__':main()
