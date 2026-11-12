#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Export stairs ablation test results to WPS/Excel compatible formats (.xlsx and .csv).
Target Session: door_diagnostics/session_20261103_184007_stairs_test4 / Frame 183
"""

import os
import csv
import xlsxwriter

OUTPUT_XLSX = '/home/yichangfeng/sru-navigation-learning/stairs_ablation_results_frame183.xlsx'
OUTPUT_CSV = '/home/yichangfeng/sru-navigation-learning/stairs_ablation_results_frame183.csv'

# Data for Table 1: Core Comparison
table1_data = [
    {
        "id": 1,
        "depth_type": "原始深度 (Raw Depth)",
        "condition": "基线 1：全 0.4m (纯短前瞻)",
        "query_dist": 0.4,
        "goal_dist": 0.4,
        "step10_vx": 0.539,
        "step10_vy": -0.064,
        "step10_wz": -0.115,
        "step10_raw_wz": -0.117,
        "avg_vx": 0.643,
        "avg_wz": -0.120,
        "status": "正常顺畅直行下楼"
    },
    {
        "id": 2,
        "depth_type": "原始深度 (Raw Depth)",
        "condition": "解耦组 1：注意力查询 0.4m + SRU目标点 1.0m",
        "query_dist": 0.4,
        "goal_dist": 1.0,
        "step10_vx": 0.689,
        "step10_vy": -0.052,
        "step10_wz": -0.064,
        "step10_raw_wz": -0.066,
        "avg_vx": 0.816,
        "avg_wz": -0.057,
        "status": "正常顺畅直行下楼（速度提升，航向更稳）"
    },
    {
        "id": 3,
        "depth_type": "原始深度 (Raw Depth)",
        "condition": "解耦组 2：注意力查询 1.0m + SRU目标点 0.4m",
        "query_dist": 1.0,
        "goal_dist": 0.4,
        "step10_vx": 0.586,
        "step10_vy": -0.135,
        "step10_wz": -0.844,
        "step10_raw_wz": -1.224,
        "avg_vx": 0.724,
        "avg_wz": -0.771,
        "status": "剧烈转向/转身拒下"
    },
    {
        "id": 4,
        "depth_type": "原始深度 (Raw Depth)",
        "condition": "基线 2：全 1.0m (纯长前瞻)",
        "query_dist": 1.0,
        "goal_dist": 1.0,
        "step10_vx": 0.988,
        "step10_vy": -0.187,
        "step10_wz": -0.789,
        "step10_raw_wz": -1.110,
        "avg_vx": 0.994,
        "avg_wz": -0.751,
        "status": "剧烈转向/转身拒下"
    },
    {
        "id": 5,
        "depth_type": "重映射深度 (Remapped)",
        "condition": "基线 1：全 0.4m (纯短前瞻 + 深度重映射)",
        "query_dist": 0.4,
        "goal_dist": 0.4,
        "step10_vx": 0.523,
        "step10_vy": -0.072,
        "step10_wz": -0.142,
        "step10_raw_wz": -0.144,
        "avg_vx": 0.620,
        "avg_wz": -0.152,
        "status": "正常顺畅直行下楼"
    },
    {
        "id": 6,
        "depth_type": "重映射深度 (Remapped)",
        "condition": "基线 2：全 1.0m (纯长前瞻 + 深度重映射)",
        "query_dist": 1.0,
        "goal_dist": 1.0,
        "step10_vx": 0.618,
        "step10_vy": -0.125,
        "step10_wz": -0.789,
        "step10_raw_wz": -1.043,
        "avg_vx": 0.721,
        "avg_wz": -0.776,
        "status": "剧烈转向/转身拒下"
    }
]

# Data for Table 2: 2D Grid Sweep (Remapped)
grid_dists = [0.2, 0.4, 0.6, 0.8, 1.0, 1.5]
grid_wz = [
    [-0.084, -0.064, -0.043, -0.023, -0.006, +0.017],
    [-0.152, -0.140, -0.128, -0.115, -0.105, -0.093],
    [-0.234, -0.233, -0.230, -0.228, -0.228, -0.230],
    [-0.364, -0.382, -0.399, -0.413, -0.425, -0.441],
    [-0.599, -0.658, -0.711, -0.753, -0.789, -0.875],
    [-0.997, -0.998, -0.998, -0.998, -0.998, -0.999]
]
grid_vx = [
    [0.444, 0.516, 0.582, 0.636, 0.676, 0.735],
    [0.455, 0.521, 0.579, 0.627, 0.667, 0.729],
    [0.447, 0.512, 0.574, 0.631, 0.675, 0.729],
    [0.420, 0.497, 0.573, 0.630, 0.666, 0.697],
    [0.389, 0.481, 0.556, 0.600, 0.618, 0.586],
    [0.450, 0.519, 0.526, 0.483, 0.419, 0.288]
]

# Data for Table 3: 10-step step-by-step
steps_data = [
    {"step": 1, "04_raw_vx": 1.23, "04_raw_wz": -1.06, "04_cmd_vx": 1.14, "04_cmd_wz": -0.39, "04_norm_c": 4.45,
                "10_raw_vx": 0.72, "10_raw_wz": -2.65, "10_cmd_vx": 0.83, "10_cmd_wz": -0.50, "10_norm_c": 3.40},
    {"step": 2, "04_raw_vx": 0.69, "04_raw_wz": +0.19, "04_cmd_vx": 0.92, "04_cmd_wz": -0.10, "04_norm_c": 6.15,
                "10_raw_vx": 0.80, "10_raw_wz": -2.54, "10_cmd_vx": 0.98, "10_cmd_wz": -0.74, "10_norm_c": 4.71},
    {"step": 3, "04_raw_vx": 0.08, "04_raw_wz": -0.07, "04_cmd_vx": 0.20, "04_cmd_wz": -0.09, "04_norm_c": 6.42,
                "10_raw_vx": 0.67, "10_raw_wz": -1.11, "10_cmd_vx": 0.89, "10_cmd_wz": -0.77, "10_norm_c": 4.93},
    {"step": 4, "04_raw_vx": 0.50, "04_raw_wz": -0.16, "04_cmd_vx": 0.64, "04_cmd_wz": -0.12, "04_norm_c": 6.47,
                "10_raw_vx": 0.43, "10_raw_wz": -1.19, "10_cmd_vx": 0.63, "10_cmd_wz": -0.80, "10_norm_c": 4.83},
    {"step": 5, "04_raw_vx": 0.47, "04_raw_wz": -0.11, "04_cmd_vx": 0.66, "04_cmd_wz": -0.11, "04_norm_c": 6.82,
                "10_raw_vx": 0.41, "10_raw_wz": -1.51, "10_cmd_vx": 0.59, "10_cmd_wz": -0.85, "10_norm_c": 4.94},
    {"step": 6, "04_raw_vx": 0.32, "04_raw_wz": -0.14, "04_cmd_vx": 0.48, "04_cmd_wz": -0.13, "04_norm_c": 6.98,
                "10_raw_vx": 0.53, "10_raw_wz": -1.28, "10_cmd_vx": 0.71, "10_cmd_wz": -0.86, "10_norm_c": 5.01},
    {"step": 7, "04_raw_vx": 0.40, "04_raw_wz": -0.15, "04_cmd_vx": 0.56, "04_cmd_wz": -0.14, "04_norm_c": 7.01,
                "10_raw_vx": 0.51, "10_raw_wz": -1.12, "10_cmd_vx": 0.70, "10_cmd_wz": -0.83, "10_norm_c": 5.07},
    {"step": 8, "04_raw_vx": 0.38, "04_raw_wz": -0.14, "04_cmd_vx": 0.54, "04_cmd_wz": -0.14, "04_norm_c": 7.10,
                "10_raw_vx": 0.45, "10_raw_wz": -1.09, "10_cmd_vx": 0.63, "10_cmd_wz": -0.81, "10_norm_c": 5.13},
    {"step": 9, "04_raw_vx": 0.36, "04_raw_wz": -0.14, "04_cmd_vx": 0.51, "04_cmd_wz": -0.14, "04_norm_c": 7.14,
                "10_raw_vx": 0.44, "10_raw_wz": -1.06, "10_cmd_vx": 0.62, "10_cmd_wz": -0.80, "10_norm_c": 5.13},
    {"step": 10,"04_raw_vx": 0.36, "04_raw_wz": -0.14, "04_cmd_vx": 0.52, "04_cmd_wz": -0.14, "04_norm_c": 7.16,
                "10_raw_vx": 0.44, "10_raw_wz": -1.04, "10_cmd_vx": 0.62, "10_cmd_wz": -0.79, "10_norm_c": 5.15},
]


def create_xlsx():
    workbook = xlsxwriter.Workbook(OUTPUT_XLSX)
    
    # Formats
    title_fmt = workbook.add_format({
        'bold': True, 'font_size': 14, 'align': 'center', 'valign': 'vcenter',
        'bg_color': '#1F4E79', 'font_color': '#FFFFFF'
    })
    subtitle_fmt = workbook.add_format({
        'italic': True, 'font_size': 10, 'align': 'left', 'valign': 'vcenter',
        'font_color': '#595959'
    })
    header_fmt = workbook.add_format({
        'bold': True, 'font_size': 11, 'align': 'center', 'valign': 'vcenter',
        'bg_color': '#2F5597', 'font_color': '#FFFFFF', 'border': 1, 'text_wrap': True
    })
    header_sec_fmt = workbook.add_format({
        'bold': True, 'font_size': 11, 'align': 'center', 'valign': 'vcenter',
        'bg_color': '#5B9BD5', 'font_color': '#FFFFFF', 'border': 1
    })
    cell_center = workbook.add_format({'align': 'center', 'valign': 'vcenter', 'border': 1})
    cell_left = workbook.add_format({'align': 'left', 'valign': 'vcenter', 'border': 1})
    cell_num3 = workbook.add_format({'align': 'right', 'valign': 'vcenter', 'border': 1, 'num_format': '0.000'})
    cell_num2 = workbook.add_format({'align': 'right', 'valign': 'vcenter', 'border': 1, 'num_format': '0.00'})
    
    # Status formats
    pass_fmt = workbook.add_format({
        'align': 'center', 'valign': 'vcenter', 'border': 1,
        'bg_color': '#E2EFDA', 'font_color': '#375623', 'bold': True
    })
    fail_fmt = workbook.add_format({
        'align': 'center', 'valign': 'vcenter', 'border': 1,
        'bg_color': '#FCE4D6', 'font_color': '#C65911', 'bold': True
    })

    # ==================== SHEET 1: 核心消融与基线对比 ====================
    ws1 = workbook.add_worksheet('十步递归核心对比')
    ws1.set_column('A:A', 6)    # 序号
    ws1.set_column('B:B', 22)   # 深度图类型
    ws1.set_column('C:C', 38)   # 实验工况说明
    ws1.set_column('D:E', 15)   # Query / Goal 距离
    ws1.set_column('F:K', 14)   # 速度指标
    ws1.set_column('L:L', 30)   # 状态评估
    
    ws1.merge_range('A1:L1', '楼梯下行第183帧：Cross-Attention解耦消融与重映射深度十步递归对比表', title_fmt)
    ws1.set_row(0, 36)
    ws1.merge_range('A2:L2', '数据来源: session_20261103_184007_stairs_test4 / Frame 183 | 控制周期: 5Hz (10步=2.0秒闭环动力学)', subtitle_fmt)
    ws1.set_row(1, 20)
    
    headers1 = [
        "序号", "深度图模式", "实验工况详细说明", "注意力Query\n前瞻距离 (m)", "SRU拼接目标点\n前瞻距离 (m)",
        "第10步前向速度\nvx (m/s)", "第10步横向飘移\nvy (m/s)", "第10步控制角速度\nwz (rad/s)",
        "第10步网络原始角速度\nraw_wz", "10步平均线速度\navg_vx (m/s)", "10步平均角速度\navg_wz (rad/s)",
        "实机动力学与运动学评估"
    ]
    ws1.set_row(2, 32)
    for col_idx, h in enumerate(headers1):
        ws1.write(2, col_idx, h, header_fmt)
        
    for r_idx, row in enumerate(table1_data):
        curr_row = 3 + r_idx
        ws1.set_row(curr_row, 24)
        ws1.write(curr_row, 0, row['id'], cell_center)
        ws1.write(curr_row, 1, row['depth_type'], cell_center)
        ws1.write(curr_row, 2, row['condition'], cell_left)
        ws1.write(curr_row, 3, row['query_dist'], cell_num2)
        ws1.write(curr_row, 4, row['goal_dist'], cell_num2)
        ws1.write(curr_row, 5, row['step10_vx'], cell_num3)
        ws1.write(curr_row, 6, row['step10_vy'], cell_num3)
        ws1.write(curr_row, 7, row['step10_wz'], cell_num3)
        ws1.write(curr_row, 8, row['step10_raw_wz'], cell_num3)
        ws1.write(curr_row, 9, row['avg_vx'], cell_num3)
        ws1.write(curr_row, 10, row['avg_wz'], cell_num3)
        
        # Status highlight
        if "正常" in row['status']:
            ws1.write(curr_row, 11, row['status'], pass_fmt)
        else:
            ws1.write(curr_row, 11, row['status'], fail_fmt)

    # Note
    note_text = (
        "【解耦测试定义说明】\n"
        "1. 解耦组 1 (Query 0.4m + Goal 1.0m)：Cross-Attention仅用0.4m近身目标查询视觉特征；进入SRU时拼接1.0m远目标点。实测wz保持-0.06rad/s稳定直行，证明转身非SRU下游目标引起。\n"
        "2. 解耦组 2 (Query 1.0m + Goal 0.4m)：Cross-Attention用1.0m长前瞻查询视觉；进入SRU时拼接0.4m近目标点。实测wz骤增至-0.84rad/s剧烈转身，证明转身完全由Query诱发的视觉注意力漂移主导。\n"
        "3. 重映射深度对比：重映射将断崖虚空深度修正为平滑坡道，降低了前向超速冲刺风险，但因未改变右侧实体墙/扶手几何，1.0m Query依然因注意力向右侧墙面转移而触发避障转身。"
    )
    ws1.merge_range(10, 0, 15, 11, note_text, workbook.add_format({
        'align': 'left', 'valign': 'top', 'text_wrap': True, 'font_size': 9, 'border': 1, 'bg_color': '#F2F2F2'
    }))

    # ==================== SHEET 2: 2D全景网格扫描 ====================
    ws2 = workbook.add_worksheet('2D网格全景扫描')
    ws2.set_column('A:A', 22)
    ws2.set_column('B:G', 14)
    
    ws2.merge_range('A1:G1', '重映射深度下：Query前瞻距离 vs SRU下游目标点距离 2D网格扫描', title_fmt)
    ws2.set_row(0, 30)
    
    # Table 2A: wz
    ws2.merge_range('A3:G3', '【表 A】第10步角速度 wz (rad/s) 全景矩阵（绝对值 < 0.25 为直行，> 0.65 为转身拒下）', header_sec_fmt)
    ws2.write('A4', 'Query前瞻 \\ 目标点', header_fmt)
    for c_idx, d in enumerate(grid_dists):
        ws2.write(3, c_idx + 1, f'{d:.1f} m', header_fmt)
        
    for r_idx, q in enumerate(grid_dists):
        curr_row = 4 + r_idx
        ws2.write(curr_row, 0, f'Query = {q:.1f} m', cell_center)
        for c_idx, val in enumerate(grid_wz[r_idx]):
            cell_fmt = fail_fmt if abs(val) >= 0.65 else (cell_num3 if abs(val) < 0.25 else pass_fmt)
            ws2.write(curr_row, c_idx + 1, val, cell_fmt)

    # Table 2B: vx
    start_r = 12
    ws2.merge_range(start_r, 0, start_r, 6, '【表 B】第10步前向线速度 vx (m/s) 全景矩阵', header_sec_fmt)
    ws2.write(start_r + 1, 0, 'Query前瞻 \\ 目标点', header_fmt)
    for c_idx, d in enumerate(grid_dists):
        ws2.write(start_r + 1, c_idx + 1, f'{d:.1f} m', header_fmt)
        
    for r_idx, q in enumerate(grid_dists):
        curr_row = start_r + 2 + r_idx
        ws2.write(curr_row, 0, f'Query = {q:.1f} m', cell_center)
        for c_idx, val in enumerate(grid_vx[r_idx]):
            ws2.write(curr_row, c_idx + 1, val, cell_num3)

    # ==================== SHEET 3: 十步演化过程对比 ====================
    ws3 = workbook.add_worksheet('十步时序递推演化过程')
    ws3.set_column('A:A', 10)
    ws3.set_column('B:K', 14)
    
    ws3.merge_range('A1:K1', '纯 0.4m（平稳下楼）与 纯 1.0m（转身拒下）十步时序状态演化对照表', title_fmt)
    ws3.set_row(0, 30)
    
    ws3.merge_range('A2:A3', '控制步数\n(每步0.2s)', header_fmt)
    ws3.merge_range('B2:F2', '基线 1：纯 0.4m 前瞻 (平稳直行下楼)', header_sec_fmt)
    ws3.merge_range('G2:K2', '基线 2：纯 1.0m 前瞻 (剧烈转身拒下)', header_sec_fmt)
    
    sub_headers = ["原始 raw_vx", "原始 raw_wz", "控制 cmd_vx", "控制 cmd_wz", "SRU记忆模长",
                   "原始 raw_vx", "原始 raw_wz", "控制 cmd_vx", "控制 cmd_wz", "SRU记忆模长"]
    for c_idx, sh in enumerate(sub_headers):
        ws3.write(2, c_idx + 1, sh, header_fmt)
        
    for r_idx, row in enumerate(steps_data):
        curr_row = 3 + r_idx
        ws3.write(curr_row, 0, f'Step {row["step"]}', cell_center)
        ws3.write(curr_row, 1, row["04_raw_vx"], cell_num2)
        ws3.write(curr_row, 2, row["04_raw_wz"], cell_num2)
        ws3.write(curr_row, 3, row["04_cmd_vx"], cell_num2)
        ws3.write(curr_row, 4, row["04_cmd_wz"], cell_num2)
        ws3.write(curr_row, 5, row["04_norm_c"], cell_num2)
        
        ws3.write(curr_row, 6, row["10_raw_vx"], cell_num2)
        ws3.write(curr_row, 7, row["10_raw_wz"], cell_num2)
        ws3.write(curr_row, 8, row["10_cmd_vx"], cell_num2)
        ws3.write(curr_row, 9, row["10_cmd_wz"], cell_num2)
        ws3.write(curr_row, 10, row["10_norm_c"], cell_num2)

    workbook.close()
    print(f"[OK] Successfully generated Excel: {OUTPUT_XLSX}")


def create_csv():
    with open(OUTPUT_CSV, 'w', encoding='utf-8-sig', newline='') as f:
        writer = csv.writer(f)
        
        # Section 1
        writer.writerow(["=== 表1：楼梯下行第183帧十步递归核心对比表 ==="])
        writer.writerow([
            "序号", "深度图模式", "实验工况说明", "注意力Query前瞻距离(m)", "SRU拼接目标点前瞻距离(m)",
            "第10步前向速度vx(m/s)", "第10步横向飘移vy(m/s)", "第10步控制角速度wz(rad/s)",
            "第10步原始角速度raw_wz", "10步平均线速度avg_vx(m/s)", "10步平均角速度avg_wz(rad/s)", "实机动力学与运动学评估"
        ])
        for r in table1_data:
            writer.writerow([
                r['id'], r['depth_type'], r['condition'], r['query_dist'], r['goal_dist'],
                f"{r['step10_vx']:.3f}", f"{r['step10_vy']:.3f}", f"{r['step10_wz']:.3f}",
                f"{r['step10_raw_wz']:.3f}", f"{r['avg_vx']:.3f}", f"{r['avg_wz']:.3f}", r['status']
            ])
            
        writer.writerow([])
        # Section 2
        writer.writerow(["=== 表2：重映射深度下 Query距离 vs 目标点距离 2D网格角速度 wz (rad/s) 矩阵 ==="])
        writer.writerow(["Query前瞻 \\ 目标点"] + [f"{d:.1f}m" for d in grid_dists])
        for r_idx, q in enumerate(grid_dists):
            writer.writerow([f"Query={q:.1f}m"] + [f"{val:+.3f}" for val in grid_wz[r_idx]])
            
        writer.writerow([])
        # Section 3
        writer.writerow(["=== 表3：十步时序递推演化对照表 (纯0.4m vs 纯1.0m) ==="])
        writer.writerow([
            "步数",
            "纯0.4m_原始raw_vx", "纯0.4m_原始raw_wz", "纯0.4m_控制cmd_vx", "纯0.4m_控制cmd_wz", "纯0.4m_SRU记忆模长",
            "纯1.0m_原始raw_vx", "纯1.0m_原始raw_wz", "纯1.0m_控制cmd_vx", "纯1.0m_控制cmd_wz", "纯1.0m_SRU记忆模长"
        ])
        for s in steps_data:
            writer.writerow([
                f"Step {s['step']}",
                s['04_raw_vx'], s['04_raw_wz'], s['04_cmd_vx'], s['04_cmd_wz'], s['04_norm_c'],
                s['10_raw_vx'], s['10_raw_wz'], s['10_cmd_vx'], s['10_cmd_wz'], s['10_norm_c']
            ])
            
    print(f"[OK] Successfully generated CSV: {OUTPUT_CSV}")


if __name__ == '__main__':
    create_xlsx()
    create_csv()
