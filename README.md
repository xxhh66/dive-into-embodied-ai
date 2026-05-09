<div align="center">
    <img src="static/img/career.webp" width="100%" alt="Dive into Embodied AI 横幅" />
</div>

<h1 align="center">Dive into Embodied AI</h1>
<p align="center"><b>具身智能入门与求职开源教程</b></p>

<p align="center">
  <a href="https://datawhalechina.github.io/dive-into-embodied-ai/"><img alt="在线阅读" src="https://img.shields.io/badge/%E5%9C%A8%E7%BA%BF%E9%98%85%E8%AF%BB-datawhalechina-blue" /></a>
  <a href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="许可协议" src="https://img.shields.io/badge/license-CC%20BY--NC--SA%204.0-lightgrey" /></a>
  <img alt="状态" src="https://img.shields.io/badge/status-Alpha-orange" />
</p>

> [!CAUTION]
> **Alpha 内测版本**:仍在迁移和重构中,部分章节是占位页,欢迎提 Issue 反馈问题或建议。

## 项目定位

面向**求职与转行**的具身智能开源教程,不追求覆盖所有论文,而是帮读者按「认知 → 项目 → 面试」三步走通一条可落地的学习路径:先建立行业与技术栈的整体认知,再在四类本体(机械臂、四足、人形、移动操作)上做能写进简历的项目,最后把知识点整理成岗位技能、简历和面经。

## 项目受众

- 想入门具身智能的应届生与在校生
- 从 ML / CV / NLP / 自动驾驶 / 传统机器人方向转入具身的工程师
- 想跳槽到更头部具身公司、需要补齐项目和面试准备的在职工程师

## 内容大纲

教程分为「总览 + 基础篇 + 实践篇 + 求职篇」三篇制,下表列出了每一篇下的主要章节与当前状态。章节名后的链接直接指向在线文档。

状态标记说明:**✅ 可用** = 章节内容完整,可直接阅读;**🚧 部分可用** = 一部分章节有内容、一部分仍是占位;**🚧 占位中** = 目录已建但只有占位页;**⏳ 待补充** = 暂未开工。

### 总览

| 章节 | 简介 | 状态 |
| :--- | :--- | :--- |
| [项目与学习路径总览](docs/overview/intro.md) | 学习主线、公司图谱、算法工程师转岗路径 | ✅ 可用 |

### 基础篇:理论知识

| 章节 | 简介 | 状态 |
| :--- | :--- | :--- |
| [具身智能入门](docs/foundations/embodied-ai-intro/) | 本体、关节、传感器、行业现状与公司图谱 | 🚧 占位中 |
| [机器人运动学与 ROS2 基础教程](docs/foundations/robotics-and-ros2/) | 坐标变换、正/逆运动学、ROS2 工程、tf2、URDF、MoveIt 2 | ✅ 可用 |
| [仿真工具基础](docs/foundations/simulation/) | Isaac Sim、MuJoCo、Gymnasium、PyBullet 快速上手 | ✅ 可用 |
| [强化学习与控制](docs/foundations/rl-for-robotics/) | MDP、DQN、PPO、SAC、DDPG/TD3,含模仿学习 | ✅ 可用 |
| [视觉语言大模型(VLM)](docs/foundations/vlm/) | Transformer、ViT、视觉编码器、多模态融合 | ✅ 可用 |
| [视觉-语言-动作大模型(VLA)](docs/foundations/vla/) | RT-1/RT-2、OpenVLA、ACT、Diffusion Policy、π 系列 | ✅ 可用 |
| [世界模型(World Model)](docs/foundations/world-model/) | 主流方案与具身应用 | 🚧 占位中 |

### 实践篇:项目实践

| 章节 | 简介 | 状态 |
| :--- | :--- | :--- |
| [机械臂](docs/practices/robot-arm/) | MuJoCo 抓放、DDPG、LeRobot 数据采集已可用;ROS2 控制、模仿学习、VLA 控制占位中 | 🚧 部分可用 |
| [四足机器人](docs/practices/quadruped/) | CS123 课程复刻 9 章可用;sim2sim、sim2real 指南占位中 | 🚧 部分可用 |
| [双足/人形](docs/practices/humanoid/) | 平衡控制、动作跟踪、任务规划 Demo | 🚧 占位中 |
| [移动操作](docs/practices/mobile-manipulation/) | 导航基础、视觉语言导航、移动操作 Demo | 🚧 占位中 |

### 求职篇:求职面试

| 章节 | 简介 | 状态 |
| :--- | :--- | :--- |
| [岗位技能拆解](docs/career/job-skill-map/) | 强化学习、VLA 等方向的技能点拆解 | 🚧 占位中 |
| [转岗路径](docs/career/transition-paths/) | 从 ML / CV / 自动驾驶 / 传统机器人切入具身 | ✅ 可用 |
| [面经与八股](docs/career/interview-questions/) | 具身方向常见面试题与高频八股 | 🚧 占位中 |
| [简历与作品集](docs/career/resume-portfolio/) | 简历结构、项目描述、GitHub 作品集 | 🚧 占位中 |
| [公司技术栈](docs/career/company-tech-stacks/) | 头部具身公司主力技术栈与方向差异 | 🚧 占位中 |
| [招聘信息](docs/career/job-listings/) | Top 具身公司在招岗位汇总 | 🚧 占位中 |
| [社区与内推](docs/career/community/) | 社群、内推渠道、开源协作入口 | 🚧 占位中 |

## 组队学习

Datawhale 会围绕本教程组织组队学习。历史与在筹备中的组队学习计划文档会集中放在 `docs/team-learning/`(施工中),包括每期的学习路线、打卡要求和对应章节的导读。

- 最新一期报名入口:施工中
- 往期学习资料归档:施工中

## 本地预览

仓库使用 **Git LFS** 存放视频和 GIF。clone 之后必须先装 `git-lfs` 再 `git lfs pull`,否则本地看到的图/视频是 pointer 文本而不是真内容。完整步骤见 [CONTRIBUTING.md](CONTRIBUTING.md#首次克隆必读)。

```bash
# 1. 装 git-lfs(每台机器只需一次)
# brew install git-lfs        # macOS
# sudo apt install git-lfs    # Ubuntu / Debian
# choco install git-lfs       # Windows

# 2. 初始化并拉取 LFS 文件
git lfs install
git lfs pull

# 3. 装依赖、起本地预览
npm install
npm run dev
```

## 贡献者名单

| 姓名 | 职责 | 简介 |
| :--- | :--- | :--- |
| 罗如意 | 项目负责人 | 智能汽车竞赛国奖&多模态顶会Oral&FunRec开源项目负责人 |
| 江季  | 项目负责人 | [蘑菇书](https://github.com/datawhalechina/easy-rl)作者 |
| 康博 | 核心贡献者 | nobl.ai 联合创始人 & 比利时根特大学访问教授|

## 关注我们

<div align=center>
<p>扫描下方二维码关注公众号:Datawhale</p>
<img src="https://raw.githubusercontent.com/datawhalechina/pumpkin-book/master/res/qrcode.jpeg" width = "180" height = "180">
</div>

## LICENSE

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="知识共享许可协议" style="border-width:0" src="https://img.shields.io/badge/license-CC%20BY--NC--SA%204.0-lightgrey" /></a><br />本作品采用<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">知识共享署名-非商业性使用-相同方式共享 4.0 国际许可协议</a>进行许可。
