# Games103Learn
Labs and notes when learning Games103. 
边学边写，放一些记录。

## 配置环境

我在Windows下使用VSCode写代码，配置的时候参考了一些资料。

- 下载安装Unity，这里Unity Hub中install下好了就结束了，记得在Unity Hub得设置（Preferences）中改一下所有下载相关地址。另外不需要用团结引擎的话请不要碰Unity Hub页面左侧栏第二个图标，否则C盘会收获一个5G大礼包。  
[Unity下载与安装](https://blog.csdn.net/Dugege007/article/details/128472571)

- 连接VSCode。VSCode部分需要下载Unity插件，它会帮你把C#、C# Dev Kit也下下来。.NET Install Tool插件可以自动帮你下载.Net，VSCode找不到的话要手动指定一下。Unity部分则需要在Project的Edit-Preferences-External Tools-External Script Editor中改变使用的编辑器。使用的参考包括：  
[分享vscode如何进行unity开发，且如何开启unity断点调试模式，并进行unity断点调试](https://blog.csdn.net/qq_36303853/article/details/144285531)  
[为VSCode配置完整的Unity开发环境，并解决无代码提示词问题](https://www.bilibili.com/opus/805683968643956744)  
[VSCode安装插件Unity后，一直弹The .NET Core SDK cannot be located](https://www.cnblogs.com/bakabird/p/17729159.html)  

- 导入unitypakage。首先在Unity Hub上创建一个新的Project，Lab1用的是3D的场景，创建一个空的3D Project，这个还挺耗时的，得耐心等等。随后会进入Unity的界面。在Assets-Import Package-Custom Package中导入作业需要的bunny.unitypackage即可。导入后会看见两个C#文件，即本次作业的两个算法文件。双击即可使用设置好的编辑器打开。

- 在Unity上跑已经写好的C#脚本。点击左侧Hierarchy-bunny-default右侧会出现Inspector，下面连接了两个C#脚本，需要跑哪个就给哪个打勾，点击界面上方播放键。  
[挂载C#脚本到游戏物体](https://blog.csdn.net/shulianghan/article/details/127890336?utm_medium=distribute.pc_relevant.none-task-blog-2~default~baidujs_baidulandingword~default-0-127890336-blog-119931281.235^v43^pc_blog_bottom_relevance_base5&spm=1001.2101.3001.4242.1&utm_relevant_index=2)

- 录屏，请同时看看评论区：[如何用Unity输出视频](https://www.bilibili.com/video/BV1Lt411H7bp/?vd_source=91cc1aa1ac04a25ac6d622053847ab41)

- 上传Github记得在每个Unity Project根目录下加Unity的.gitignore，否则上传的东西太多了。  
[git管理Unity项目的正确方式](https://blog.csdn.net/qq_51326491/article/details/144239802)

## Lab 1
用rigid body dynamics + impluse做碰撞的效果：


https://github.com/user-attachments/assets/b3cfaec7-cc00-43bf-b328-e7736339d63b


用shape matching的效果：


https://github.com/user-attachments/assets/9c427ad5-7e39-4268-83a6-6f3fb69cadfc


Lab1中rigid body dynamics + Impulse和shape matching两种方法都使用了restitution ($\mu_N$) 和friction ($\mu_T$) 两个系数来控制碰撞，其中shape matching需要restitution显著大于前一种方法。猜测是因为在bunny这个相对比较凸的形状上，每一次碰撞的顶点数较少，一个点对shape matching的影响比较小，需要放大或者改变最小二乘使用的权重。而rigid body dynamics + Impulse实际上已经隐式的保证了碰撞点速度正确，因此正常取个[0, 1]的restitution即可。

## Lab 2
Implicit IP，弹性势能只有弹簧能量：


https://github.com/user-attachments/assets/d640bc00-7e8b-40e4-843d-7bc0e88bbc5a


PBD：


https://github.com/user-attachments/assets/33d3cc06-2f8a-402c-83ed-d65b5e423e96


这里PBD没有实现stiffness与它需要的和迭代次数有关的调整。

## Lab 3
stvk能量：


https://github.com/user-attachments/assets/df9f3fdc-7fc4-4e5a-bf4a-d4316cfa209d


ppt上Hyperelastic公式应该是错的，得自己推一下。
```math
E = \frac{1}{2}(F^T F-I),\ \Psi(F) = \mu \Vert E \Vert_F^2 + \frac{\lambda}{2}\text{tr}(E)^2,\ P = F(2\mu E + \lambda \text{tr}(E) I).
```
如果做SVD分解，
```math
F = U\Sigma V^T,\ \Sigma = \text{diag}\{\lambda_0, \lambda_1, \lambda_2\},\ P = U(\mu \Sigma(\Sigma^2 - I) + \frac{\lambda}{2}\text{tr}(\Sigma^2 - I)\Sigma)V^T.
```
SVD分解还是挺慢的，明显感觉到比直接使用矩阵乘法慢。显式积分弹性体的硬度不能太大，否则会炸。

## Lab 4


https://github.com/user-attachments/assets/cd7d7281-382e-47ed-882e-94d0970e49ee


