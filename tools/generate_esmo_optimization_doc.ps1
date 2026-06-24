param(
    [string]$OutputPath = ""
)

$ErrorActionPreference = "Stop"

if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $OutputPath = Join-Path $PSScriptRoot "..\docs\双电机eSMO从0.7pu高速瓶颈到0.9pu稳定运行的优化过程与性能验证.docx"
}

$OutputPath = [System.IO.Path]::GetFullPath($OutputPath)
$repoRoot = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot ".."))
$figureRoot = Join-Path $repoRoot "figures\v9.0\2026-06-23_full_angle_validation"

$word = $null
$doc = $null

function Set-RangeFont {
    param($Range, [double]$Size = 10.5, [string]$EastAsia = "宋体", [string]$Latin = "Times New Roman")
    $Range.Font.Name = $Latin
    $Range.Font.NameFarEast = $EastAsia
    $Range.Font.Size = $Size
}

function Add-Paragraph {
    param(
        [string]$Text,
        [switch]$NoIndent,
        [switch]$Center,
        [switch]$Bold,
        [double]$Size = 10.5,
        [double]$SpaceAfter = 6
    )
    $p = $doc.Paragraphs.Add()
    $p.Range.Text = $Text
    Set-RangeFont -Range $p.Range -Size $Size
    $p.Range.Bold = [int]$Bold.IsPresent
    $p.Alignment = if ($Center) { 1 } else { 3 }
    $p.Format.LineSpacingRule = 1
    $p.Format.FirstLineIndent = if ($NoIndent -or $Center) { 0 } else { 21 }
    $p.Format.SpaceAfter = $SpaceAfter
    $p.Range.InsertParagraphAfter()
    return $p
}

function Add-Heading {
    param([string]$Text, [int]$Level)
    $p = $doc.Paragraphs.Add()
    $p.Range.Text = $Text
    $styleId = switch ($Level) { 1 { -2 } 2 { -3 } default { -4 } }
    $p.Range.Style = $doc.Styles.Item($styleId)
    $size = switch ($Level) { 1 { 16 } 2 { 14 } default { 12 } }
    Set-RangeFont -Range $p.Range -Size $size -EastAsia "黑体"
    $p.Range.Bold = 1
    $p.Format.SpaceBefore = if ($Level -eq 1) { 12 } else { 8 }
    $p.Format.SpaceAfter = 6
    $p.Range.InsertParagraphAfter()
    return $p
}

function Add-Bullet {
    param([string]$Text)
    $p = $doc.Paragraphs.Add()
    $p.Range.Text = $Text
    Set-RangeFont -Range $p.Range
    $p.Range.ListFormat.ApplyBulletDefault()
    $p.Format.LeftIndent = 21
    $p.Format.FirstLineIndent = -10.5
    $p.Format.LineSpacingRule = 1
    $p.Format.SpaceAfter = 3
    $p.Range.InsertParagraphAfter()
}

function Add-Table {
    param([string[]]$Headers, [object[]]$Rows, [double[]]$Widths = @())
    $range = $doc.Bookmarks.Item("\endofdoc").Range
    $table = $doc.Tables.Add($range, $Rows.Count + 1, $Headers.Count)
    $table.Borders.Enable = 1
    $table.AllowAutoFit = $true
    $table.Rows.Item(1).Range.Bold = 1
    $table.Rows.Item(1).Shading.BackgroundPatternColor = 14277081

    for ($c = 1; $c -le $Headers.Count; $c++) {
        $table.Cell(1, $c).Range.Text = $Headers[$c - 1]
        if ($Widths.Count -eq $Headers.Count) {
            $table.Columns.Item($c).PreferredWidth = $Widths[$c - 1]
        }
    }
    for ($r = 0; $r -lt $Rows.Count; $r++) {
        for ($c = 0; $c -lt $Headers.Count; $c++) {
            $table.Cell($r + 2, $c + 1).Range.Text = [string]$Rows[$r][$c]
        }
    }
    Set-RangeFont -Range $table.Range -Size 9
    $table.Range.ParagraphFormat.SpaceAfter = 0
    $table.Range.ParagraphFormat.Alignment = 1
    $table.Range.InsertParagraphAfter()
    $after = $doc.Paragraphs.Add()
    $after.Format.SpaceAfter = 6
    $after.Range.InsertParagraphAfter()
    return $table
}

function Add-Figure {
    param([string]$FileName, [string]$Caption)
    $path = Join-Path $figureRoot $FileName
    if (-not (Test-Path -LiteralPath $path)) {
        throw "Figure not found: $path"
    }
    $p = $doc.Paragraphs.Add()
    $p.Alignment = 1
    $shape = $p.Range.InlineShapes.AddPicture($path, $false, $true)
    if ($shape.Width -gt 430) {
        $ratio = 430.0 / $shape.Width
        $shape.Width = 430
        $shape.Height = $shape.Height * $ratio
    }
    $p.Range.InsertParagraphAfter()
    Add-Paragraph -Text $Caption -NoIndent -Center -Size 9 -SpaceAfter 8 | Out-Null
}

try {
    $word = New-Object -ComObject Word.Application
    $word.Visible = $false
    $word.DisplayAlerts = 0
    $doc = $word.Documents.Add()

    $section = $doc.Sections.Item(1)
    $section.PageSetup.PaperSize = 7
    $section.PageSetup.TopMargin = 72
    $section.PageSetup.BottomMargin = 72
    $section.PageSetup.LeftMargin = 72
    $section.PageSetup.RightMargin = 72
    $section.Footers.Item(1).PageNumbers.Add(1) | Out-Null

    $normal = $doc.Styles.Item(-1)
    Set-RangeFont -Range $normal -Size 10.5
    $normal.ParagraphFormat.LineSpacingRule = 1
    $normal.ParagraphFormat.SpaceAfter = 6

    $title = $doc.Paragraphs.Add()
    $title.Range.Text = "双电机 eSMO 系统从 0.7 pu 高速瓶颈到 0.9 pu 稳定运行的优化过程与性能验证"
    $title.Range.Style = $doc.Styles.Item(-63)
    Set-RangeFont -Range $title.Range -Size 20 -EastAsia "黑体"
    $title.Range.Bold = 1
    $title.Alignment = 1
    $title.Format.SpaceAfter = 18
    $title.Range.InsertParagraphAfter()

    Add-Paragraph -Text "项目：Dual-Motor-SMO-Control（v9.0）" -NoIndent -Center -Size 12 -SpaceAfter 6 | Out-Null
    Add-Paragraph -Text "整理日期：2026 年 6 月 24 日" -NoIndent -Center -Size 11 -SpaceAfter 24 | Out-Null

    Add-Heading -Text "摘  要" -Level 1 | Out-Null
    Add-Paragraph -Text "针对双电机 eSMO 无位置传感器控制系统在 0.7 pu 以上转速区间出现电压裕量不足、速度停滞以及约 0.8 pu 附近停转的问题，本文采用分层诊断和保守迭代方法，对电流环电压限幅、速度反馈链路、锁相环高速带宽及角度系统偏差进行了联合优化。首先统一 QEP 旁路参考与 FOC 电角度零位，并通过短窗口 RAM 日志分离观测误差、速度反馈误差和电压饱和现象；随后在稳定启动与接管基线上，采用停机写入、运行保持的 d/q 轴静态 PI 限幅，提高高速 q 轴电压调节余量；在 0.70–0.80 pu 区间平滑提升 PLL 比例增益，并以补偿后的 PLL 原始角构造速度反馈；最后依据 0.2–0.9 pu 全电角度实验建立速度分段偏置表，同时关闭会造成高速过补偿的 thetaErr 前馈。实验表明，系统可在弱磁控制关闭的条件下稳定运行至 0.9 pu，八个速度点的最大绝对电角度误差均低于 10°，最坏值为 5.10°，去均值角度纹波 RMS 为 0.43°–0.69°。结果说明，高速电压调节能力、PLL 跟踪带宽和速度反馈质量共同决定可运行转速上限，而参考坐标一致性与速度相关偏置校正决定稳态角度精度。" | Out-Null
    Add-Paragraph -Text "关键词：扩展滑模观测器；无位置传感器控制；永磁同步电机；锁相环；电压利用率；角度偏置校正；双电机系统" -NoIndent -Bold | Out-Null

    $doc.Words.Last.InsertBreak(7)
    Add-Heading -Text "目录" -Level 1 | Out-Null
    $tocRange = $doc.Bookmarks.Item("\endofdoc").Range
    $doc.TablesOfContents.Add($tocRange, $true, 1, 3) | Out-Null
    $doc.Words.Last.InsertBreak(7)

    Add-Heading -Text "1 研究背景与问题定义" -Level 1 | Out-Null
    Add-Paragraph -Text "本项目面向双永磁同步电机实验平台，将 eSMO 估计的电角度和速度用于 FCL 矢量控制。前期工作已经完成开环强拖、eSMO 接管和双电机闭环运行，但可用速度范围长期受限于 0.7 pu 附近。当指令继续提高到 0.8–0.9 pu 时，系统表现为转速不能继续上升、减速停转，个别版本还伴随电机 0 接管后短暂停转、反转、双电机不同步和高速噪声增大。" | Out-Null
    Add-Paragraph -Text "典型故障点的特征是：速度 PI 输出及 Iq 指令继续增大，q 轴电压输出却已经达到 PI 上限，实际速度稳定在约 0.766 pu；这表明故障并非单纯由速度指令斜坡或机械负载造成，而是高速区电压调节余量不足与观测器相位跟踪能力下降共同作用。与此同时，早期 eSMO-QEP 角度差还混入了 QEP 物理 index 零位与 FOC 磁链零位不一致的问题，因此不能直接把全部相位差解释为滑模观测误差。" | Out-Null

    Add-Table -Headers @("阶段", "速度表现", "关键观测", "初步判断") -Rows @(
        @("早期高速测试", "0.7 pu 可运行；接近 0.8 pu 后停转", "q 轴 PI 使用率接近或达到 100%", "高速电压调节余量不足"),
        @("0.9 pu 指令停滞", "实际速度约 0.766 pu", "速度 PI/Iq 指令饱和，vq 达上限", "速度环继续加力但执行端受限"),
        @("部分激进改版", "可短时提速但启动恶化", "电机 0 停顿、反转、不同步、噪声增大", "动态限幅与接管状态发生耦合"),
        @("v9.0 最终版本", "0.2–0.9 pu 稳态运行", "0.9 pu 物理调制率仍低于 100%", "高速带宽与电压余量得到平衡")
    ) | Out-Null

    Add-Heading -Text "2 诊断方法与实验设计" -Level 1 | Out-Null
    Add-Heading -Text "2.1 分层诊断原则" -Level 2 | Out-Null
    Add-Paragraph -Text "优化过程中没有直接大幅调整 Kslide 或同时引入多种补偿，而是按「坐标参考—启动接管—速度反馈—电流环电压—PLL 相位—稳态系统偏差」的顺序逐层排查。这样能够避免将 QEP 零位错误、速度量化、PI 饱和和观测器本体误差互相混淆。" | Out-Null
    Add-Bullet -Text "坐标层：确认 QEP 旁路角使用 alignment 建立的 FOC 零位，物理 index 只用于保持该零位。"
    Add-Bullet -Text "状态层：确认 lsw、接管混合、速度 PI 种入及重复 STOP/RUN 后状态复位的一致性。"
    Add-Bullet -Text "执行层：分别记录 vd、vq、轴 PI 使用率、设计电压预算和物理调制率。"
    Add-Bullet -Text "观测层：分别记录 PLL 原始角、延迟补偿角、最终控制角、eSMO/QEP 速度及 Eq_mag、thetaErr。"
    Add-Bullet -Text "统计层：用完整电角度短窗口计算误差均值、最大绝对值和去均值 RMS。"

    Add-Heading -Text "2.2 RAM 日志与测试矩阵" -Level 2 | Out-Null
    Add-Paragraph -Text "最终验证在 0.2、0.3、0.4、0.5、0.6、0.7、0.8 和 0.9 pu 八个速度点进行。每组在 ENC_CALIBRATION_DONE 稳态后手动触发，采样率为 500 Hz，记录 96 点，对应 0.192 s。日志包含速度指令、eSMO/QEP 角度与速度、IqRef、IqFbk、IdFbk、vd、vq、电压使用率、Eq_mag 和 thetaErr。96 点数组放入 RAMGS6，既覆盖多个电周期，又避免持续日志对 ISR 和 RAM 造成额外负担。" | Out-Null
    Add-Paragraph -Text "角度误差采用环形归一化：eθ = wrap(θeSMO - θQEP)，并换算为电角度。稳态角度质量同时使用最大绝对误差 max|eθ| 和去均值 RMS 评价，后者用于区分固定偏置与周期纹波。物理调制率定义为 M = sqrt(vd² + vq²) / maxModIndex。" -NoIndent -Center | Out-Null

    Add-Heading -Text "3 从速度瓶颈到 0.9 pu 的关键改进" -Level 1 | Out-Null
    Add-Heading -Text "3.1 以稳定版本为基线，隔离启动与高速问题" -Level 2 | Out-Null
    Add-Paragraph -Text "多轮实验表明，同时修改接管角混合、Iq 限幅、双电机同步和高速补偿会放大耦合风险。部分版本虽然提高了高速电压权限，却引入电机 0 在 ENC_CALIBRATION_DONE 后停顿甚至反转，以及高速噪声增大。最终采用 2026-06-22 稳定版本作为启动和接管基线，只逐项恢复已在台架上验证的高速功能。该策略把「能可靠启动」和「能稳定高速运行」分成两个独立验收门槛。" | Out-Null

    Add-Heading -Text "3.2 静态 d/q 轴 PI 限幅与物理调制率分离" -Level 2 | Out-Null
    Add-Paragraph -Text "原设计中 d 轴和 q 轴 PI 上限分别约为 0.5·maxModIndex 和 0.8·maxModIndex，其名义合成预算为 sqrt(0.5²+0.8²)=0.943·maxModIndex。高速轻载时 q 轴反电动势补偿需求显著增加，而 d 轴实际输出通常较小；过小的 d 轴权限及运行时动态缩放会使轴限幅、速度环和启动接管产生不必要耦合。最终将 d/q 轴尺度设为 0.75/0.80，并仅在 MOTOR_STOP 时写入，MOTOR_RUN 期间保持固定。" | Out-Null
    Add-Paragraph -Text "需要强调，0.75/0.80 是矩形轴限幅，其理论角点超过圆形调制边界，并不自动保证任意 vd、vq 组合都满足物理电压约束。因此代码同时保留两套指标：轴 PI/设计预算使用率用于判断控制器是否顶到软件限幅；M = sqrt(vd²+vq²)/maxModIndex 用于判断逆变器物理调制裕量。最终 0.9 pu 工况中，q 轴接近软件限幅，但实际 d 轴较小，物理调制率仍未达到 100%，从而既释放了高速 q 轴电压能力，又避免把诊断超限误判为真实过调制。" | Out-Null

    Add-Heading -Text "3.3 基于补偿 PLL 原始角的速度反馈重构" -Level 2 | Out-Null
    Add-Paragraph -Text "最终 FOC 角 esmoAnglePu 还包含接管混合、固定偏置和输出侧速度分段校正。若直接对该角差分，角度修正可能进入速度环并形成附加纹波。参考官方 eSMO 路径，当前版本对补偿后的 PLL 原始角 esmoRawAnglePu 使用 SPDFR 得到速度环反馈，并在启动或接管过渡期间保持原 eSMO 速度反馈和状态种入。该改动使速度估计链与角度输出校正解耦。" | Out-Null

    Add-Heading -Text "3.4 高速 PLL 比例增益分段提升" -Level 2 | Out-Null
    Add-Paragraph -Text "随着转速和电压利用率提高，观测相位滞后及相位扰动对 FOC 的影响增大。当前版本在速度指令绝对值超过 0.70 pu 后开始提高 PLL Kp，在 0.80 pu 达到完整增益，最高采用基础 Kp 的 1.25 倍，并设置 Kp≤7.0 的硬上限。线性过渡避免在 0.70 pu 附近产生参数突变，增益上限则限制高频噪声放大。该措施是最终版本唯一默认启用的高速观测增强项。" | Out-Null

    Add-Table -Headers @("参数", "最终值", "作用") -Rows @(
        @("ESMO_PLL_KP_BOOST_START_PU", "0.70", "高速增益开始介入"),
        @("ESMO_PLL_KP_BOOST_FULL_PU", "0.80", "达到完整高速增益"),
        @("esmoPllKpHighSpeedGain", "1.25", "提高高速 PLL 跟踪带宽"),
        @("ESMO_PLL_KP_HARD_MAX", "7.00", "限制噪声放大和参数失控"),
        @("esmoIdPiLimitSf", "0.75", "释放 d 轴软件电压权限"),
        @("esmoIqPiLimitSf", "0.80", "保留 q 轴高速反电动势补偿能力"),
        @("esmoThetaErrFFGain", "0.00", "默认关闭过补偿前馈"),
        @("flagEnableFWC", "0", "本阶段不启用弱磁")
    ) | Out-Null

    Add-Heading -Text "4 从角度偏差到全电角度误差小于 10°" -Level 1 | Out-Null
    Add-Heading -Text "4.1 QEP 参考零位统一与重复性修复" -Level 2 | Out-Null
    Add-Paragraph -Text "早期对比将物理 index 位置误作 QEP 电角度零点，使 eSMO 最终角与 QEP 参考角之间出现约百余电角度的固定坐标差。恢复原 FCL/QEP 标定逻辑后，由 alignment 建立 FOC 零位，index 仅用于保存和重载该零位，0.2 pu 下角度误差从几十度量级下降到约 1°。随后又修复 STOP/RUN 后观测器状态和角度交接状态不一致的问题，使同速重复实验的误差均值差缩小到约 0.06°–0.16°。这些工作为后续偏置标定提供了可重复的参考系。" | Out-Null

    Add-Heading -Text "4.2 速度分段偏置表" -Level 2 | Out-Null
    Add-Paragraph -Text "全电角度日志表明，误差随电角度变化的周期纹波较小，而均值随速度呈非线性变化。因此没有继续增大 Kslide，也没有使用单一全局角度偏置或单一延迟系数，而是在 0.2–0.9 pu 设置 8 个速度断点并线性插值。补偿从 0.15 pu 以下保持为零，避免进入强拖和接管过程；补偿只施加在估计角输出侧，不改写 PLL 积分状态和接管种入。" | Out-Null
    Add-Paragraph -Text "该方法本质上属于实验标定型系统误差补偿。论文中应将其表述为「速度相关稳态偏差校正」，而不是 eSMO 本体在所有工况下天然达到相同精度。为避免过拟合，后续论文实验宜使用独立重复数据、不同负载和冷热机数据验证标定表的可迁移性。" | Out-Null

    Add-Heading -Text "4.3 thetaErr 前馈的否定性结果" -Level 2 | Out-Null
    Add-Paragraph -Text "曾尝试对低通后的 PLL thetaErr 施加 0.55 倍输出角前馈，以补偿高速相位滞后。但 0.9 pu 全电角度实验表明，该项与既有速度延迟补偿、固定角度偏置和速度分段偏置叠加后产生约 27° 过补偿。最终保留 Watch 可调接口用于对照实验，但默认增益为 0。这个结果说明，多种相位补偿不能仅凭单项物理含义叠加，必须以最终 FOC 控制角相对独立参考角的闭环实验为准。" | Out-Null

    Add-Heading -Text "5 未采用方案及其原因" -Level 1 | Out-Null
    Add-Table -Headers @("方案", "实验现象", "最终处理") -Rows @(
        @("运行中动态缩放 d/q PI 限幅", "与启动、接管和同步状态耦合，出现停顿、反转或噪声增大", "改为停机设置、运行固定"),
        @("thetaErr 前馈增益 0.55", "0.9 pu 约 27° 过补偿", "接口保留，默认关闭"),
        @("同时修改接管与高速策略", "故障归因困难，双电机行为不一致", "回到稳定基线后逐项恢复"),
        @("默认启用弱磁", "尚未完成双电机硬件整定，可能影响启动和 Id", "FWC 默认关闭，不计入本阶段结果"),
        @("继续大幅调整 Kslide", "现有误差主要为可重复直流偏置，纹波已较小", "保持 Kslide=0.55 标定状态"),
        @("长时间高频全量日志", "RAM 和 ISR 负担过大", "RAMGS6 中 500 Hz、96 点手动短窗口")
    ) | Out-Null

    Add-Heading -Text "6 最终实验结果" -Level 1 | Out-Null
    Add-Table -Headers @("速度", "误差范围 / 电角度", "均值 / 电角度", "去均值 RMS / 电角度", "最大绝对误差 / 电角度") -Rows @(
        @("0.2 pu", "-1.08°–+2.18°", "+0.56°", "0.69°", "2.18°"),
        @("0.3 pu", "-3.76°–-1.79°", "-2.69°", "0.53°", "3.76°"),
        @("0.4 pu", "-0.75°–+0.90°", "+0.13°", "0.44°", "0.90°"),
        @("0.5 pu", "+0.69°–+2.47°", "+1.62°", "0.51°", "2.47°"),
        @("0.6 pu", "-0.76°–+1.31°", "+0.09°", "0.48°", "1.31°"),
        @("0.7 pu", "+0.89°–+2.70°", "+1.71°", "0.43°", "2.70°"),
        @("0.8 pu", "-0.32°–+1.36°", "+0.49°", "0.49°", "1.36°"),
        @("0.9 pu", "-5.10°–-3.50°", "-4.39°", "0.46°", "5.10°")
    ) | Out-Null
    Add-Paragraph -Text "八个速度点均满足 max|eθ|≤10°，最坏结果为 0.9 pu 下 5.10°，相对限值仍有 4.90° 裕量。全速域去均值角度纹波 RMS 为 0.43°–0.69°；Iq 跟踪误差 RMS 不超过 2.3×10⁻³ pu，Id RMS 不超过 2.7×10⁻³ pu。0.9 pu 下系统能够稳态运行且不触发 tripFlagDMC。" | Out-Null

    Add-Figure -FileName "fig1_speed_tracking.png" -Caption "图 1  0.2–0.9 pu 稳态速度跟踪结果（通道定义以固件字段为准）"
    Add-Figure -FileName "fig3_angle_summary.png" -Caption "图 2  不同速度下的电角度估计误差统计"
    Add-Figure -FileName "fig5_voltage.png" -Caption "图 3  不同速度下的电压利用率与裕量"

    Add-Heading -Text "7 机理讨论" -Level 1 | Out-Null
    Add-Paragraph -Text "高速停转的直接表现是 q 轴电压控制权限耗尽，但仅放宽限幅并不能保证稳定，因为 eSMO 的相位误差会改变 Park 变换方向，使同样的电压矢量不能有效产生期望转矩。反之，仅增加 PLL 带宽也无法突破软件电压上限。因此最终 0.9 pu 能力来自三者的协同：静态轴限幅提供执行端余量，高速 PLL Kp 提高角度跟踪能力，基于 PLL 原始角的速度反馈避免输出角校正污染速度环。" | Out-Null
    Add-Paragraph -Text "稳态角度性能的改善则具有不同机理。QEP 零位统一消除了坐标定义误差，重复启停状态复位消除了运行间随机固定偏差，速度分段表补偿了剩余可重复的速度相关系统误差。由于去均值 RMS 始终低于 0.7°，说明当前台架的主要误差成分是低频或直流偏置，而非显著的电角度周期性抖振。" | Out-Null

    Add-Heading -Text "8 可用于论文正文的归纳表述" -Level 1 | Out-Null
    Add-Paragraph -Text "针对 eSMO 无位置传感器控制在高速区出现的转速饱和问题，本文首先基于 d/q 轴电压输出、PI 限幅使用率和物理调制率建立分层诊断指标。实验发现，当速度指令提高至 0.8–0.9 pu 时，q 轴电压调节量接近软件限幅，速度环输出继续增加但转速不再上升。为此，在保持启动和接管逻辑不变的基础上，将 d/q 轴电压限幅改为停机写入、运行固定的非对称静态限幅，并在 0.70–0.80 pu 区间平滑提高 PLL 比例增益。同时，以补偿后的 PLL 原始角构造速度反馈，实现速度估计与最终 FOC 角校正的解耦。上述措施在弱磁控制关闭条件下将稳定运行范围扩展至 0.9 pu。" | Out-Null
    Add-Paragraph -Text "在角度精度方面，本文先统一 QEP 参考角与 FOC 电角度零位，并修复重复启停后的观测器状态不一致。全电角度实验显示，残余误差以速度相关直流偏置为主，周期纹波较小，因此采用基于八个速度断点的线性插值偏置补偿，而未继续提高滑模增益。0.2–0.9 pu 实验中，最大绝对电角度误差为 5.10°，去均值误差 RMS 为 0.43°–0.69°，验证了所提联合优化方法在当前实验条件下的有效性。" | Out-Null

    Add-Heading -Text "9 结论、边界与后续工作" -Level 1 | Out-Null
    Add-Paragraph -Text "本阶段完成了从 0.7 pu 高速瓶颈到 0.9 pu 稳态运行的扩速，并在全电角度范围内满足最大误差不超过 10°的目标。有效改进集中在电压调节余量、高速 PLL 带宽、速度反馈解耦和稳态偏置校正；动态限幅、默认弱磁和 thetaErr 前馈均未作为最终性能来源。" | Out-Null
    Add-Bullet -Text "当前结论仅覆盖正转、当前母线电压、轻载及本次温度条件。"
    Add-Bullet -Text "速度偏置表是经验标定结果，应通过独立重复实验检验，避免训练数据与验证数据重合。"
    Add-Bullet -Text "下一阶段应增加反转、负载阶跃、冷热机、长时温升和双电机同步扰动实验。"
    Add-Bullet -Text "若继续扩展基速以上范围，应单独启用并整定弱磁控制，同时重新验证角度偏置表。"
    Add-Bullet -Text "论文中应区分‘补偿后的系统角度精度’与‘未补偿观测器本体误差’，两者不能混称。"

    Add-Heading -Text "附录 A 关键观测变量" -Level 1 | Out-Null
    Add-Table -Headers @("变量", "物理含义", "论文用途") -Rows @(
        @("esmoSpeedPu / qepSpeedPu", "eSMO 与 QEP 速度", "速度估计和闭环跟踪对比"),
        @("esmoAngleErrPu", "最终 eSMO 角减 QEP FOC 参考角", "电角度误差统计"),
        @("pi_id.out / pi_iq.out", "d/q 轴电压指令", "判断电流环执行需求"),
        @("esmoVdUsePct / esmoVqUsePct", "各轴相对软件 PI 限幅使用率", "定位轴饱和"),
        @("esmoModUsePct", "物理电压矢量相对 maxModIndex 的比例", "判断实际过调制风险"),
        @("Eq_mag", "估算反电动势幅值", "评价观测信号强度"),
        @("thetaErr", "PLL 相位误差信号", "评价 PLL 跟踪状态"),
        @("esmoPllKpApplied", "实际应用的 PLL Kp", "验证高速增益调度"),
        @("tripFlagDMC", "驱动故障标志", "运行安全性验收")
    ) | Out-Null

    $doc.TablesOfContents.Item(1).Update()
    $doc.Fields.Update() | Out-Null
    $outputDir = Split-Path -Parent $OutputPath
    if (-not (Test-Path -LiteralPath $outputDir)) {
        New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
    }
    $doc.SaveAs2($OutputPath, 16)
    $doc.Close($false)
    $doc = $null
    $word.Quit()
    $word = $null

    Write-Output $OutputPath
}
finally {
    if ($null -ne $doc) {
        $doc.Close($false)
    }
    if ($null -ne $word) {
        $word.Quit()
    }
    [GC]::Collect()
    [GC]::WaitForPendingFinalizers()
}

