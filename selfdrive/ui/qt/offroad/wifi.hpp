#pragma once

#include <QWidget>
#include <QButtonGroup>
#include <QVBoxLayout>
#include <QStackedWidget>
#include <QTimer>

#include "wifiManager.hpp"
#include "input_field.hpp"


class WifiUI : public QWidget {
  Q_OBJECT

private:
  WifiManager* wifi;
  const int networks_per_page = 8;

  QStackedWidget* swidget;
  QVBoxLayout* vlayout;
  QWidget * wifi_widget;

  InputField *a;
  QEventLoop loop;
  QTimer * timer;
  QString text;
  QButtonGroup *connectButtons;

  void connectToNetwork(Network n);
  QString getStringFromUser();

public:
  int page;
  explicit WifiUI(QWidget *parent = 0);

private slots:
  void handleButton(QAbstractButton* m_button);
  void refresh();
  void receiveText(QString text);
  void wrongPassword(QString ssid);

  void prevPage();
  void nextPage();

signals:
  void openKeyboard();
  void closeKeyboard();
};
