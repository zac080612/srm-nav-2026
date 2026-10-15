#!/usr/bin/env python3
# 生成「调试困难与解决」PPT（纯标准库，不需要 python-pptx）
# 用法：python3 tools/generate_ppt.py
import zipfile
import os

P = 'http://schemas.openxmlformats.org/presentationml/2006/main'
A = 'http://schemas.openxmlformats.org/drawingml/2006/main'
R = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'
CT = 'http://schemas.openxmlformats.org/package/2006/content-types'
REL = 'http://schemas.openxmlformats.org/package/2006/relationships'

def esc(s):
    return s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')

# ---- 每页：一个标题文本框 + 一个内容文本框 ----
def slide_xml(title, lines, code=None):
    """lines: 普通要点列表；code: 可选代码块（等宽字体）"""
    body_paras = ''
    for ln in lines:
        body_paras += f'<a:p><a:r><a:rPr lang="zh-CN" sz="1600"/><a:t>{esc(ln)}</a:t></a:r></a:p>'
    if code:
        for ln in code:
            body_paras += ('<a:p><a:r><a:rPr lang="en-US" sz="1400" '
                           'typeface="Consolas"/><a:t>' + esc(ln) + '</a:t></a:r></a:p>')

    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sld xmlns:a="{A}" xmlns:r="{R}" xmlns:p="{P}">
  <p:cSld><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
    <p:sp>
      <p:nvSpPr><p:cNvPr id="2" name="Title"/><p:cNvSpPr><a:spLocks noGrp="1"/></p:cNvSpPr><p:nvPr/></p:nvSpPr>
      <p:spPr><a:xfrm><a:off x="457200" y="274320"/><a:ext cx="8229600" cy="914400"/></a:xfrm>
        <a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr>
      <p:txBody><a:bodyPr/><a:lstStyle/>
        <a:p><a:r><a:rPr lang="zh-CN" sz="2800" b="1"/><a:t>{esc(title)}</a:t></a:r></a:p>
      </p:txBody>
    </p:sp>
    <p:sp>
      <p:nvSpPr><p:cNvPr id="3" name="Body"/><p:cNvSpPr><a:spLocks noGrp="1"/></p:cNvSpPr><p:nvPr/></p:nvSpPr>
      <p:spPr><a:xfrm><a:off x="457200" y="1371600"/><a:ext cx="8229600" cy="5029200"/></a:xfrm>
        <a:prstGeom prst="rect"><a:avLst/></a:prstGeom></p:spPr>
      <p:txBody><a:bodyPr/><a:lstStyle/>{body_paras}</p:txBody>
    </p:sp>
  </p:spTree></p:cSld>
  <p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sld>'''

def content_types(n):
    overrides = ''.join(
        f'<Override PartName="/ppt/slides/slide{i}.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slide+xml"/>'
        for i in range(1, n + 1))
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="{CT}">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/ppt/presentation.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.presentation.main+xml"/>
  <Override PartName="/ppt/slideMasters/slideMaster1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideMaster+xml"/>
  <Override PartName="/ppt/slideLayouts/slideLayout1.xml" ContentType="application/vnd.openxmlformats-officedocument.presentationml.slideLayout+xml"/>
  <Override PartName="/ppt/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>
  {overrides}
</Types>'''

root_rels = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{REL}">
  <Relationship Id="rId1" Type="{R}/officeDocument" Target="ppt/presentation.xml"/>
</Relationships>'''

def presentation_xml(n):
    sld_ids = ''.join(f'<p:sldId id="{255 + i}" r:id="rId{i + 2}"/>' for i in range(1, n + 1))
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:presentation xmlns:a="{A}" xmlns:r="{R}" xmlns:p="{P}">
  <p:sldMasterIdLst><p:sldMasterId id="2147483648" r:id="rId1"/></p:sldMasterIdLst>
  <p:sldIdLst>{sld_ids}</p:sldIdLst>
  <p:sldSz cx="9144000" cy="6858000"/>
  <p:notesSz cx="6858000" cy="9144000"/>
</p:presentation>'''

def presentation_rels(n):
    rels = ('<Relationship Id="rId1" Type="%s/slideMaster" Target="slideMasters/slideMaster1.xml"/>'
            '<Relationship Id="rId2" Type="%s/slide" Target="slides/slide1.xml"/>' % (R, R))
    for i in range(2, n + 1):
        rels += f'<Relationship Id="rId{i+1}" Type="{R}/slide" Target="slides/slide{i}.xml"/>'
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{REL}">{rels}</Relationships>'''

slide_master = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldMaster xmlns:a="{A}" xmlns:r="{R}" xmlns:p="{P}">
  <p:cSld><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
  <p:clrMap bg1="lt1" tx1="dk1" bg2="lt2" tx2="dk2" accent1="accent1" accent2="accent2" accent3="accent3" accent4="accent4" accent5="accent5" accent6="accent6" hlink="hlink" folHlink="folHlink"/>
  <p:sldLayoutIdLst><p:sldLayoutId id="2147483649" r:id="rId1"/></p:sldLayoutIdLst>
</p:sldMaster>'''

slide_master_rels = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{REL}">
  <Relationship Id="rId1" Type="{R}/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>
  <Relationship Id="rId2" Type="{R}/theme" Target="../theme/theme1.xml"/>
</Relationships>'''

slide_layout = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<p:sldLayout xmlns:a="{A}" xmlns:r="{R}" xmlns:p="{P}" type="blank">
  <p:cSld name="Blank"><p:spTree>
    <p:nvGrpSpPr><p:cNvPr id="1" name=""/><p:cNvGrpSpPr/><p:nvPr/></p:nvGrpSpPr>
    <p:grpSpPr><a:xfrm><a:off x="0" y="0"/><a:ext cx="0" cy="0"/><a:chOff x="0" y="0"/><a:chExt cx="0" cy="0"/></a:xfrm></p:grpSpPr>
  </p:spTree></p:cSld>
  <p:clrMapOvr><a:masterClrMapping/></p:clrMapOvr>
</p:sldLayout>'''

slide_layout_rels = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{REL}">
  <Relationship Id="rId1" Type="{R}/slideMaster" Target="../slideMasters/slideMaster1.xml"/>
</Relationships>'''

theme = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<a:theme xmlns:a="{A}" name="Simple">
  <a:themeElements>
    <a:clrScheme name="Simple">
      <a:dk1><a:srgbClr val="000000"/></a:dk1><a:lt1><a:srgbClr val="FFFFFF"/></a:lt1>
      <a:dk2><a:srgbClr val="000000"/></a:dk2><a:lt2><a:srgbClr val="FFFFFF"/></a:lt2>
      <a:accent1><a:srgbClr val="4472C4"/></a:accent1><a:accent2><a:srgbClr val="ED7D31"/></a:accent2>
      <a:accent3><a:srgbClr val="A5A5A5"/></a:accent3><a:accent4><a:srgbClr val="FFC000"/></a:accent4>
      <a:accent5><a:srgbClr val="5B9BD5"/></a:accent5><a:accent6><a:srgbClr val="70AD47"/></a:accent6>
      <a:hlink><a:srgbClr val="0563C1"/></a:hlink><a:folHlink><a:srgbClr val="954F72"/></a:folHlink>
    </a:clrScheme>
    <a:fontScheme name="Simple">
      <a:majorFont><a:latin typeface="Calibri"/><a:ea typeface=""/><a:cs typeface=""/></a:majorFont>
      <a:minorFont><a:latin typeface="Calibri"/><a:ea typeface=""/><a:cs typeface=""/></a:minorFont>
    </a:fontScheme>
    <a:fmtScheme name="Simple"><a:fillStyleLst/><a:lnStyleLst/><a:effectStyleLst/><a:bgFillStyleLst/></a:fmtScheme>
  </a:themeElements>
</a:theme>'''

def slide_rels():
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="{REL}">
  <Relationship Id="rId1" Type="{R}/slideLayout" Target="../slideLayouts/slideLayout1.xml"/>
</Relationships>'''

def build(slides, out_path):
    n = len(slides)
    with zipfile.ZipFile(out_path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', content_types(n))
        z.writestr('_rels/.rels', root_rels)
        z.writestr('ppt/presentation.xml', presentation_xml(n))
        z.writestr('ppt/_rels/presentation.xml.rels', presentation_rels(n))
        z.writestr('ppt/slideMasters/slideMaster1.xml', slide_master)
        z.writestr('ppt/slideMasters/_rels/slideMaster1.xml.rels', slide_master_rels)
        z.writestr('ppt/slideLayouts/slideLayout1.xml', slide_layout)
        z.writestr('ppt/slideLayouts/_rels/slideLayout1.xml.rels', slide_layout_rels)
        z.writestr('ppt/theme/theme1.xml', theme)
        for i, (t, lines, code) in enumerate(slides, 1):
            z.writestr(f'ppt/slides/slide{i}.xml', slide_xml(t, lines, code))
            z.writestr(f'ppt/slides/_rels/slide{i}.xml.rels', slide_rels())

slides = [
    ("调试中遇到的困难与解决", ["SRM 校内赛 · 导航赛道（8 字绕桩）", "", "姓名：____________"], None),
    ("为什么不用 Nav2，自己写状态机", [
        "任务太简单：两根固定桩、一条固定八字，不需要全局规划/代价地图/行为树",
        "实车没有里程计：Nav2 依赖 odom，而实车只有 Livox 雷达，靠桩当信标定位",
        "成本不划算：Nav2 要装一堆依赖、调一堆参数（约 20~30h）；自己写状态机约 10h",
        "可控性：7 个状态、切换条件明确，核心算法纯 C++ 可单测、模拟实车复用",
    ], None),
    ("问题 11：headless 下 Gazebo 实时因子低（车动得慢）", [
        "现象：无 GUI 验证时，车移动速度约为命令速度的 1/11",
        "原因：软件渲染下 Gazebo 实时因子 RTF≈0.06，仿真慢于真实时间",
        "解决：这是无 GPU 环境的性能现象，不是代码问题",
        "　→ GUI 终端跑 run_sim.sh 时 RTF≈1，速度正常",
    ], None),
    ("问题 12：planar_move 插件 dynamic 模型被摩擦拖慢", [
        "现象：RTF=1，但车实际速度只有命令速度的约 1/20",
        "原因：插件用 SetLinearVel 只设瞬时速度，dynamic 模型被地面摩擦拖慢",
        "解决：把车模型设为 kinematic，精确控速、不受摩擦",
        "　→ 绕桩模拟本就不需要物理（绕行半径 0.55 > 桩+车半径 0.35）",
    ], ['<kinematic>true</kinematic>']),
    ("问题 13：节点退出后车失控继续跑（协议无超时保护）", [
        "现象：节点退出后，车还在继续转圈停不下来",
        "原因：planar_move 一直沿用最后一帧速度；协议本身没有超时保护",
        "解决：两道安全网——",
        "　① fake_mcu 失联保护：200ms 没收到帧就发零速",
        "　② launch 的 OnProcessExit→Shutdown：任一节点退出就关掉整个模拟",
        "实车结论：真下位机不做超时保护，上车必须另配看门狗！",
    ], None),
    ("问题 14：状态机换桩连跳（绕完下桩直接回停车位）", [
        "现象：绕完下桩到交叉点后直接回停车位，跳过绕上桩",
        "原因：Cross→OrbitUp 时重置 angle_consumed_，旧累积角让 OrbitUp 立即触发",
        "解决：换桩时不重置 angle_consumed_，等累积角回零再重新计数",
    ], None),
    ("问题 15：绕下桩只绕一半就离开（last_bearing_ 初始值）", [
        "现象：绕下桩约 195° 就切走，而不是绕满 330°",
        "原因：last_bearing_ 初始为 0，进绕桩第一帧虚增了入点方位角约 135°",
        "解决：加 was_orbiting_ 标志，刚进绕桩时只初始化、不累积角度",
    ], None),
    ("问题 16：orbit_step 切向与径向符号写反", [
        "现象：车顺时针绕桩（应逆时针），且半径越绕越大、撞桩",
        "原因：切向和径向两个符号都写反了（物理方向反了但单测自洽，没拦住）",
        "解决：切向 t = ccw ? (uy,-ux) : (-uy,ux)，径向 +k_r*(r-R)",
        "教训：靠诊断日志对比「累积角 vs 实际方位角」才定位到",
    ], ['const double tx = p.ccw ? uy : -uy;',
        'const double ty = p.ccw ? -ux : ux;',
        'const double radial = p.k_r * (r - p.R);']),
    ("我对导航的感受", ["", "", "", "", "", "", ""], None),
]

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '调试困难与解决.pptx')
build(slides, out)
print("已生成:", os.path.abspath(out))
