# CodeCraft2024

## 代码协作

从`master`分支拉去最新代码
```bash
git pull origin master
```
每次开始修改代码时签出到新的分支
```bash
git checkout -b new_branch_name
```
提交代码到新分支
```bash
git add .
git commit -m "message"
git push origin new_branch_name
```
在github页面提交合并代码到`master`分支请求。

如果代码有冲突，在本地解决冲突后重新再提交到`new_branch_name`分支，新提交的内容会更新在合并请求中。

让其他人审核代码，同一合并分支请求。

合并分支后可以在[github action页面](https://github.com/jankiny/CodeCraft2024/actions)查看master分支代码运行结果。