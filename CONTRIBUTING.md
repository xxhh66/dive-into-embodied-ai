# Contributing

## 首次克隆（必读）

仓库里 GIF、视频、PSD、ZIP 等大文件全部走 Git LFS。**没装 git-lfs 直接 clone，本地看到的图/视频会是几行文本（pointer 文件），不是真内容。**

```bash
# 1. 装 git-lfs（每台机器只需一次）
brew install git-lfs        # macOS
# sudo apt install git-lfs  # Ubuntu / Debian
# choco install git-lfs     # Windows

# 2. clone 之后在仓库目录里
git lfs install   # 给当前仓库挂上 lfs 过滤器
git lfs pull      # 把 LFS 文件真正下载下来

# 3. 验证：随便挑一个视频看大小，几 MB 就对了
ls -lh docs/foundations/simulation/figs/isaac-sim-cube-demo.mp4
```

如果某天发现页面里图/视频坏了，第一反应跑一次 `git lfs pull`。

## 本地预览

```bash
npm install
npm run dev          # 日常预览，不会动 LFS
PULL_LFS=1 npm run dev   # 想顺带刷新 LFS 文件再用这个
```

## 提交规则

- **位图用 WebP**。不要提交新的 `png` / `jpg` / `jpeg`，先跑 `npm run assets:webp`。
- **大文件走 LFS**。GIF、视频、PSD、ZIP 等不要把真实二进制提交进 Git 历史。
- **招聘岗位数据不要提交进仓库**。后续由外部服务、定时任务或 artifact 发布。
- **提交前至少跑** `npm run assets:webp:check` 和 `npm run build`。
