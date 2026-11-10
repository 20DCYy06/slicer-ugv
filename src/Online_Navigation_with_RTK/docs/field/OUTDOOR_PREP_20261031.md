# 旧 PREP 已废止（2026-10-31）

本页原有“默认Gmapping/D455、odom默认入口、0.20m/s覆盖、GNSS只作预览”的说明已不适用，旧命令全部停止使用。

当前唯一操作单：[户外RTK初始化、采点与返回](OUTDOOR_RTK_20261031.md)。以有效工作树和安装版本为准；本轮F1/F2与capture版本354169a已完成ARM64部署和软件验证，户外实车验收尚未进行。

现行默认是DWA + 0x16、现有局部EKF与全局robot_localization；深度/Gmapping显式候选保持关闭；线速上限0.4m/s。初始化先用`geographic:=false`的odom导航，成功并停车后只切换导航为默认map档，定位始终保持运行。不得用旧页坐标示例代替当前配准和采点文件。

修改前的本页未提交内容已原样保存到本轮本地证据`.test-results/geographic-fixes-20261031/before/docs/field/OUTDOOR_PREP_20261031.md`，没有重置或丢弃用户已有工作。
