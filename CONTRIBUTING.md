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

## 本机 SSH 配置（可选）

`git push` 会先触发 `.husky/pre-push` → `git lfs pre-push`，过程中会开多次 SSH 连接。如果没把 key 挂进 ssh-agent，每次都会问 passphrase。macOS 一次性解决：

```bash
ssh-add --apple-use-keychain ~/.ssh/id_rsa
```

并在 `~/.ssh/config` 里给 `Host github.com` 加上：

```
Host github.com
  AddKeysToAgent yes
  UseKeychain yes
  IdentityFile ~/.ssh/id_rsa
```

之后 push 就不会反复问 passphrase 了，重启后也会自动从 Keychain 恢复。
