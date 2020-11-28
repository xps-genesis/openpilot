#include <QDebug>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QPushButton>
#include <QLineEdit>

#include "wifi.hpp"


void clearLayout(QLayout* layout) {
  while (QLayoutItem* item = layout->takeAt(0)) {
    if (QWidget* widget = item->widget()){
      widget->deleteLater();
    }
    if (QLayout* childLayout = item->layout()) {
      clearLayout(childLayout);
    }
    delete item;
  }
}

WifiUI::WifiUI(QWidget *parent) : QWidget(parent) {
  wifi = new WifiManager;
  QObject::connect(wifi, SIGNAL(wrongPassword(QString)), this, SLOT(wrongPassword(QString)));

  QVBoxLayout * top_layout = new QVBoxLayout;
  swidget = new QStackedWidget;

  // Networks page
  wifi_widget = new QWidget;
  vlayout = new QVBoxLayout;
  wifi_widget->setLayout(vlayout);
  swidget->addWidget(wifi_widget);

  // Keyboard page
  a = new InputField();
  QObject::connect(a, SIGNAL(emitText(QString)), this, SLOT(receiveText(QString)));
  swidget->addWidget(a);
  swidget->setCurrentIndex(0);

  top_layout->addWidget(swidget);
  setLayout(top_layout);
  a->setStyleSheet(R"(
    QLineEdit {
      background-color: #114265;
    }
  )");

  // Update network list
  timer = new QTimer(this);
  QObject::connect(timer, SIGNAL(timeout()), this, SLOT(refresh()));
  timer->start(2000);

  // Scan on startup
  wifi->request_scan();
  QLabel* scanning = new QLabel(this);
  scanning->setText("Scanning for networks");
  vlayout->addWidget(scanning);
  refresh();
  page = 0;
}

void WifiUI::refresh() {
  if (!this->isVisible()) {
    return;
  }

  wifi->request_scan();
  wifi->refreshNetworks();

  clearLayout(vlayout);

  connectButtons = new QButtonGroup(this);
  QObject::connect(connectButtons, SIGNAL(buttonClicked(QAbstractButton*)), this, SLOT(handleButton(QAbstractButton*)));

  int i = 0;
  int countWidgets = 0;
  for (Network &network : wifi->seen_networks){
    QHBoxLayout *hlayout = new QHBoxLayout;
    if(page * networks_per_page <= i && i < (page + 1) * networks_per_page){
      // SSID
      hlayout->addSpacing(50);
      hlayout->addWidget(new QLabel(QString::fromUtf8(network.ssid)));

      // strength indicator
      unsigned int strength_scale = network.strength / 17;
      QPixmap pix("../assets/images/network_" + QString::number(strength_scale) + ".png");
      QLabel *icon = new QLabel();
      icon->setPixmap(pix.scaledToWidth(100, Qt::SmoothTransformation));
      icon->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
      hlayout->addWidget(icon);
      hlayout->addSpacing(20);

      // connect button
      QPushButton* btn = new QPushButton(network.connected == ConnectedType::CONNECTED ? "Connected" : (network.connected == ConnectedType::CONNECTING ? "Connecting" : "Connect"));
      btn->setFixedWidth(300);
      btn->setDisabled(network.connected == ConnectedType::CONNECTED || network.connected == ConnectedType::CONNECTING || network.security_type == SecurityType::UNSUPPORTED);
      hlayout->addWidget(btn);
      hlayout->addSpacing(20);

      connectButtons->addButton(btn, i);

      QWidget * w = new QWidget;
      w->setLayout(hlayout);
      vlayout->addWidget(w);
      w->setStyleSheet(R"(
        QLabel {
          font-size: 50px;
        }
        QPushButton:enabled {
          background-color: #114265;
        }
        QPushButton:disabled {
          background-color: #323C43;
        }
        * {
          background-color: #114265;
        }
      )");
      countWidgets+=1;
    }
    i+=1;
  }

  //Pad vlayout to prevert oversized network widgets in case of low visible network count
  for(int i = countWidgets ; i < networks_per_page ; i++){
    QWidget * w = new QWidget;
    vlayout->addWidget(w);
  }
  
  QHBoxLayout *prev_next_buttons = new QHBoxLayout;
  QPushButton* prev = new QPushButton("Previous");
  prev->setEnabled(page);
  prev->setFixedHeight(100);
  QPushButton* next = new QPushButton("Next");
  next->setFixedHeight(100);
  
  //If there are more visible networks then we can show, enable going to next page
  if(wifi->seen_networks.size() > (page + 1) * networks_per_page){
    next->setEnabled(true);
  }else{
    next->setDisabled(true);
  }
  QObject::connect(prev, SIGNAL(released()), this, SLOT(prevPage()));
  QObject::connect(next, SIGNAL(released()), this, SLOT(nextPage()));
  prev_next_buttons->addWidget(prev);
  prev_next_buttons->addWidget(next);

  QWidget * w = new QWidget;
  w->setLayout(prev_next_buttons);
  w->setStyleSheet(R"(
    QPushButton:enabled {
      background-color: #114265;
    }
    QPushButton:disabled {
      background-color: #323C43;
    }
    * {
      background-color: #114265;
    }
  )");
  vlayout->addWidget(w);
}

void WifiUI::handleButton(QAbstractButton* button) {
  QPushButton* btn = static_cast<QPushButton*>(button);
  Network n = wifi->seen_networks[connectButtons->id(btn)];

  a->label->setText("Enter password for \"" + n.ssid  + "\"");
  connectToNetwork(n);
}

void WifiUI::connectToNetwork(Network n){
  timer->stop();
  if(n.security_type == SecurityType::OPEN){
    wifi->connect(n);
  } else if (n.security_type == SecurityType::WPA){
    QString password = getStringFromUser();
    if(password.size()){
      wifi->connect(n, password);
    }
  }
  refresh();
  timer->start();
}

QString WifiUI::getStringFromUser(){
  emit openKeyboard();
  swidget->setCurrentIndex(1);
  loop.exec();
  emit closeKeyboard();
  swidget->setCurrentIndex(0);
  return text;
}

void WifiUI::receiveText(QString t) {
  loop.quit();
  text = t;
}


void WifiUI::wrongPassword(QString ssid){
  if(loop.isRunning()){
    return;
  }
  for(Network n : wifi->seen_networks){
    if(n.ssid == ssid){
      a->label->setText("Wrong password for \"" + n.ssid +"\"");
      connectToNetwork(n);
    }
  }
}

void WifiUI::prevPage() {
  page--;
  refresh();
}
void WifiUI::nextPage() {
  page++;
  refresh();
}
