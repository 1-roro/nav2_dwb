#!/usr/bin/env python3
"""
BehaviorTree.CPP v3 XML 을 그림(png / svg)으로 그린다. graphviz 의 dot 이 필요하다.

  sudo apt install graphviz fonts-nanum
  python3 draw_bt.py ../behavior_trees/mission.xml ../doc/mission_bt.png

- 노드 색: 제어 노드(회색), 데코레이터(흰색 점선), Nav2 BT 노드(파랑), 그 외 액션(주황)
- 액션 노드 앞의 번호는 실행 순서 (모두 성공했을 때)
- 노드 바로 앞에 XML 주석이 있으면 그 노드(와 자식들)를 점선 상자로 묶고 주석을 제목으로 쓴다
"""

import argparse
import html
import os
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ET

CONTROL = {
    'Sequence': '→', 'SequenceStar': '→', 'ReactiveSequence': '→', 'PipelineSequence': '→',
    'Fallback': '?', 'ReactiveFallback': '?', 'RecoveryNode': '?',
    'Parallel': '⇉', 'RoundRobin': '↻', 'IfThenElse': '?', 'WhileDoElse': '?',
}
DECORATOR = {
    'RetryUntilSuccessful', 'Repeat', 'Inverter', 'ForceSuccess', 'ForceFailure', 'Timeout',
    'KeepRunningUntilFailure', 'RateController', 'DistanceController', 'SpeedController',
    'GoalUpdater', 'SingleTrigger', 'Delay', 'SubTree',
}
NAV2 = {
    'NavigateToPose', 'NavigateThroughPoses', 'ComputePathToPose', 'ComputePathThroughPoses',
    'FollowPath', 'SmoothPath', 'Spin', 'Wait', 'BackUp', 'DriveOnHeading', 'ClearEntireCostmap',
    'RemovePassedGoals', 'TruncatePath', 'GoalReached', 'IsBatteryLow', 'IsStuck',
    'TimeExpired', 'DistanceTraveled', 'TransformAvailable', 'IsPathValid',
}

STYLE = {   # fill, border, border style
    'control': ('#f0efec', '#52514e', 'rounded,filled'),
    'decorator': ('#ffffff', '#52514e', 'rounded,filled,dashed'),
    'nav2': ('#cde2fb', '#2a78d6', 'rounded,filled'),
    'action': ('#fbdccf', '#eb6834', 'rounded,filled'),
}
INK, INK2, EDGE = '#0b0b0b', '#52514e', '#8a8984'
CIRCLED = '①②③④⑤⑥⑦⑧⑨⑩⑪⑫⑬⑭⑮⑯⑰⑱⑲⑳'


def kind_of(tag):
    if tag in CONTROL:
        return 'control'
    if tag in DECORATOR:
        return 'decorator'
    if tag in NAV2:
        return 'nav2'
    return 'action'


def esc(text):
    return html.escape(text, quote=True)


class Drawer:

    def __init__(self, font):
        self.font = font
        self.lines = []
        self.count = 0
        self.order = 0

    def label(self, el, kind):
        rows = []
        tag = el.tag
        name = el.get('name', '')
        if kind == 'control':
            rows.append(f'<b>{esc(CONTROL[tag])} {esc(tag)}</b>')
        elif kind == 'decorator':
            rows.append(f'<b>{esc(tag)}</b>')
        else:
            self.order += 1
            num = CIRCLED[self.order - 1] if self.order <= len(CIRCLED) else f'({self.order})'
            rows.append(f'<b>{num} {esc(tag)}</b>')
        if name:
            rows.append(f'<font color="{INK2}">{esc(name)}</font>')
        for key, value in el.attrib.items():
            if key == 'name':
                continue
            rows.append(f'<font point-size="10">{esc(key)} = {esc(value)}</font>')
        cells = ''.join(f'<tr><td align="center">{r}</td></tr>' for r in rows)
        return f'<<table border="0" cellborder="0" cellspacing="0" cellpadding="1">{cells}</table>>'

    def node(self, el):
        self.count += 1
        nid = f'n{self.count}'
        kind = kind_of(el.tag)
        fill, border, style = STYLE[kind]
        self.lines.append(
            f'{nid} [label={self.label(el, kind)}, style="{style}", fillcolor="{fill}", '
            f'color="{border}", penwidth=1.5];')
        return nid

    def subtree(self, el):
        nid = self.node(el)
        for i, (child_el, comment) in enumerate(children_with_comments(el)):
            if comment:
                self.lines.append(f'subgraph cluster_{self.count} {{')
                self.lines.append(
                    f'label=<{cluster_title(comment)}>; labelloc=b; labeljust=l; style="rounded,dashed"; '
                    f'color="#c3c2b7"; fontcolor="{INK2}"; fontsize=13; margin=12;')
                cid = self.subtree(child_el)
                self.lines.append('}')
            else:
                cid = self.subtree(child_el)
            if i > 0 and 'Fallback' in el.tag:   # 앞 자식이 실패해야 실행된다
                self.lines.append(
                    f'{nid} -> {cid} [style=dashed, label=<<font point-size="10" '
                    f'color="{INK2}">실패 시</font>>];')
            else:
                self.lines.append(f'{nid} -> {cid};')
        return nid

    def dot(self, root):
        legend = ''.join(
            f'<td bgcolor="{STYLE[k][0]}" border="1" color="{STYLE[k][1]}">&nbsp;{esc(t)}&nbsp;</td>'
            f'<td>&nbsp;&nbsp;</td>'
            for k, t in [('control', '제어 노드'), ('decorator', '데코레이터'),
                         ('nav2', 'Nav2 BT 노드 (서버에 요청)'), ('action', '직접 만든 노드')])
        caption = (f'<table border="0" cellspacing="0" cellpadding="4">'
                   f'<tr><td><table border="0" cellspacing="0" cellpadding="4"><tr>{legend}</tr>'
                   f'</table></td></tr>'
                   f'<tr><td><font color="{INK2}" point-size="11">번호: 자식을 왼쪽부터 tick 하는 순서.'
                   f'  Parallel 의 자식은 동시에, Fallback 의 뒤 자식(점선)은 앞 자식이 실패할 때만 실행'
                   f'</font></td></tr></table>')
        self.subtree(root)
        head = [
            'digraph BT {',
            f'graph [rankdir=TB, ordering=out, nodesep=0.25, ranksep=0.45, '
            f'fontname="{self.font}", bgcolor="white", pad=0.3, labelloc=b, '
            f'label=<{caption}>];',
            f'node [shape=box, fontname="{self.font}", fontsize=12, fontcolor="{INK}", margin="0.12,0.06"];',
            f'edge [arrowhead=none, color="{EDGE}", penwidth=1.2, tailport=s, headport=n];',
        ]
        return '\n'.join(head + self.lines + ['}'])


def cluster_title(comment):
    text = ' '.join(comment.split())
    return '<br align="left"/>'.join(esc(line) for line in textwrap.wrap(text, 34)) + \
        '<br align="left"/>'


def children_with_comments(el):
    """자식 요소를 (요소, 바로 앞의 주석) 으로 돌려준다."""
    out = []
    pending = None
    for child in el:
        if child.tag is ET.Comment:
            pending = child.text.strip()
            continue
        out.append((child, pending))
        pending = None
    return out


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('xml')
    parser.add_argument('output', help='.png or .svg')
    parser.add_argument('--tree', help='BehaviorTree ID (default: main_tree_to_execute)')
    parser.add_argument('--font', default='NanumGothic')
    parser.add_argument('--dpi', type=int, default=110)
    args = parser.parse_args()

    root = ET.parse(args.xml, parser=ET.XMLParser(target=ET.TreeBuilder(insert_comments=True)))
    root = root.getroot()
    tree_id = args.tree or root.get('main_tree_to_execute')
    trees = [t for t in root.findall('BehaviorTree') if tree_id in (None, t.get('ID'))]
    if not trees:
        sys.exit(f'no BehaviorTree "{tree_id}" in {args.xml}')
    top = [c for c, _ in children_with_comments(trees[0])]

    drawer = Drawer(args.font)
    src = drawer.dot(top[0])
    fmt = os.path.splitext(args.output)[1].lstrip('.') or 'png'
    subprocess.run(['dot', f'-T{fmt}', f'-Gdpi={args.dpi}', '-o', args.output],
                   input=src.encode(), check=True)
    print(f'wrote {args.output}')


if __name__ == '__main__':
    main()
