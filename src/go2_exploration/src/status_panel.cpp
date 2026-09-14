#include <rviz/panel.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QTextBrowser>
#include <QTimer>
#include <QScrollBar>
#include <QVBoxLayout>
#include <QMainWindow>
#include <QDockWidget>
#include <algorithm>
#include <mutex>

namespace go2_exploration {
class StatusPanel : public rviz::Panel {
 public:
  explicit StatusPanel(QWidget* parent=nullptr):rviz::Panel(parent) {
    auto* layout=new QVBoxLayout(this);
    title_=new QLabel(QString::fromUtf8("GO2 自主探索 · 等待状态数据"),this);
    title_->setWordWrap(true);
    title_->setStyleSheet("font-size:17px;font-weight:bold;padding:8px;color:#243b53;");
    text_=new QTextBrowser(this);text_->setOpenExternalLinks(false);
    text_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Expanding);
    layout->addWidget(title_);layout->addWidget(text_);setMinimumWidth(380);setMaximumWidth(520);
    subscriber_=nh_.subscribe<std_msgs::String>("/exploration/dashboard",1,
      [this](const std_msgs::String::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(mutex_);payload_=QByteArray::fromStdString(message->data);stamp_=ros::WallTime::now();
      });
    auto* timer=new QTimer(this);
    connect(timer,&QTimer::timeout,this,[this] {render();});timer->start(500);
    QTimer::singleShot(1500,this,[this] {
      QWidget* ancestor=parentWidget();
      while(ancestor && !qobject_cast<QDockWidget*>(ancestor))ancestor=ancestor->parentWidget();
      auto* dock=qobject_cast<QDockWidget*>(ancestor);
      if(!dock)return;
      auto* main=qobject_cast<QMainWindow*>(dock->window());
      if(!main)return;
      main->addDockWidget(Qt::RightDockWidgetArea,dock);
      main->resizeDocks({dock},{470},Qt::Horizontal);
      for(auto* other:main->findChildren<QDockWidget*>())
        if(other!=dock && main->dockWidgetArea(other)==Qt::LeftDockWidgetArea)
          main->resizeDocks({other},{320},Qt::Horizontal);
    });
  }
 private:
  void render() {
    QByteArray data;ros::WallTime stamp;
    {std::lock_guard<std::mutex> lock(mutex_);data=payload_;stamp=stamp_;}
    if(stamp.isZero() || (ros::WallTime::now()-stamp).toSec()>2.5) {
      title_->setText(QString::fromUtf8("状态数据未连接或已过期"));
      title_->setStyleSheet("font-size:17px;font-weight:bold;padding:8px;color:#b42318;");
      return;
    }
    const auto doc=QJsonDocument::fromJson(data);
    if(!doc.isObject())return;
    const auto root=doc.object();const auto state=root["state"].toString();
    title_->setText(root["title"].toString());
    title_->setStyleSheet(QString("font-size:17px;font-weight:bold;padding:8px;color:%1;")
      .arg(state=="FAULT_STOPPED"?"#b42318":"#135e4b"));
    QString html="<html><body style='font-family:sans-serif;font-size:12px;color:#223344;'><table width='100%' cellspacing='0' cellpadding='5'>";
    const char* colors[]={"#146349","#9a6700","#b42318","#65758b"};
    for(const auto& entry:root["rows"].toArray()) {
      const auto row=entry.toObject();const int level=std::max(0,std::min(3,row["level"].toInt(3)));
      html+="<tr><td style='background:#edf2f7;border-bottom:1px solid #d4dde5;'><b>"+row["name"].toString().toHtmlEscaped()+"</b>";
      if(row["age"].isDouble())html+=QString(" <span style='color:#718096;'>%1 s</span>").arg(row["age"].toDouble(),0,'f',1);
      html+="<br><span style='color:"+QString(colors[level])+";'>"+row["value"].toString().toHtmlEscaped()+"</span></td></tr>";
    }
    html+="</table><p style='color:#65758b;'>此面板只读。API 成功不等于已迈步；灰色表示数据缺失或过期。</p></body></html>";
    const int scroll=text_->verticalScrollBar()->value();
    text_->setHtml(html);text_->verticalScrollBar()->setValue(scroll);
  }
  ros::NodeHandle nh_;ros::Subscriber subscriber_;std::mutex mutex_;
  QByteArray payload_;ros::WallTime stamp_;QLabel* title_;QTextBrowser* text_;
};
}
PLUGINLIB_EXPORT_CLASS(go2_exploration::StatusPanel,rviz::Panel)
