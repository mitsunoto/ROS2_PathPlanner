#include "path_controller/gui.hpp"
#include <QDialog>
#include <QToolBar>
#include <QToolButton>
#include <QLineEdit>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include <QRegularExpression>
#include <QRegularExpressionValidator>
#include <QMessageBox>
#include <thread>
#include "path_controller/snail_utils.hpp"

/**
 * Создает главное окно с виджетом MapWidget и панелью управления.
 * @param widget     ссылочный указатель на создаваемый MapWidget
 * @param width      ширина окна
 * @param height     высота окна
 * @param goal_client   клиент параметров для узла goal_handler
 * @param robot_client  клиент параметров для узла robot_controller
 * @return указатель на QMainWindow
 */
QMainWindow* createMainWindow(
    MapWidget*& widget,
    int width,
    int height,
    std::shared_ptr<rclcpp::SyncParametersClient> goal_param_client,
    std::shared_ptr<rclcpp::SyncParametersClient> robot_param_client
) {
  widget = new MapWidget();
  widget->setMinimumSize(width, height);

  auto mainWindow = new QMainWindow();
  mainWindow->setWindowTitle("Map Visualizer");

  // Центральный виджет
  QWidget* central = new QWidget(mainWindow);
  QHBoxLayout* hLayout = new QHBoxLayout(central);
  hLayout->setContentsMargins(0, 0, 0, 0);
  hLayout->addWidget(widget);
  mainWindow->setCentralWidget(central);

  // Панель управления
  QToolBar* toolBar = new QToolBar("Controls", mainWindow);
  toolBar->setMovable(false);
  mainWindow->addToolBar(Qt::TopToolBarArea, toolBar);


  // Кнопка "Создать робота"
  QToolButton* posButton = new QToolButton(mainWindow);
  posButton->setText("Создать робота");
  posButton->setStyleSheet(
      "QToolButton { border: 2px solid #555; border-radius: 4px; padding: 4px; }"
      "QToolButton:hover { background-color: #ddd; }"
  );
  posButton->setAutoRaise(false);
  toolBar->addWidget(posButton);

  // Лямбда для валидатора координат
  auto makeCoordValidator = [](QObject* parent){
    // 1) буква+цифра (A1, B3, …), опционально 2 числа 1–25 через пробел
    QRegularExpression re("^([A-Za-z]\\d)(?:\\s+([1-9]|1[0-9]|2[0-5]))?(?:\\s+([1-9]|1[0-9]|2[0-5]))?$");
    return new QRegularExpressionValidator(re, parent);
  };

  // Функционал кнопки "Создать робота"
  QObject::connect(posButton, &QToolButton::clicked,
    [mainWindow, robot_param_client, makeCoordValidator]() {
      QDialog dialog(mainWindow);
      dialog.setWindowTitle("Координаты робота");
  
      QVBoxLayout* layout = new QVBoxLayout(&dialog);
      QLabel* inputLabel = new QLabel(
        "Введите координаты в виде <span style='color:red'>*</span>x<span style='color:red'>*</span>y k l"
      );
      inputLabel->setTextFormat(Qt::RichText);
      layout->addWidget(inputLabel);
  
      QLineEdit* inputEdit = new QLineEdit();
      inputEdit->setPlaceholderText("Например: A1 5 23"); 
      inputEdit->setValidator(makeCoordValidator(&dialog));
      layout->addWidget(inputEdit);
  
        QLabel* desc = new QLabel(
            "<span style='color:red'>*</span> обязательные переменные<br>"
            "x — абсцисса на карте верхнего уровня<br>"
            "y — ордината на карте верхнего уровня<br>"
            "k — номер дочерней ячейки по \"улитке\" на карте<br>"
            "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; среднего уровня (от 1 до 25)<br>"
            "l — номер дочерней ячейки по \"улитке\" на карте<br>"
            "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; нижнего уровня (от 1 до 25)"
        );
        desc->setTextFormat(Qt::RichText);
        layout->addWidget(desc);

    QDialogButtonBox* buttonBox =
      new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    layout->addWidget(buttonBox);

    QObject::connect(buttonBox, &QDialogButtonBox::accepted, 
      [&dialog, &inputEdit, robot_param_client]() {
        QString text = inputEdit->text().trimmed();
        if (text.isEmpty()) {
          QMessageBox::warning(&dialog, "Ошибка", "Введите хотя бы координату верхнего уровня");
          return;
        }
      // Разбиваем на 1–3 части
      QStringList parts = text.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
      // Координаты верхнего уровня: буква+цифра
      QString coarse = parts[0].toUpper();
      QChar letter = coarse[0];
      bool ok;
      int coarseY = coarse.mid(1).toInt(&ok);
      if (!ok) {
        QMessageBox::warning(&dialog, "Ошибка", "Неверный формат верхнего уровня");
        return;
      }
      int coarseX = letter.unicode() - QChar('A').unicode(); 
  
      // параметры сред/мелкого уровня
      int k = (parts.size() >= 2) ? parts[1].toInt(&ok) : 0; 
      if (parts.size() >= 2 && (!ok || k < 1 || k > 25)) {
        QMessageBox::warning(&dialog, "Ошибка", "Номер среднего уровня 1–25");
        return;
      }
      int l = (parts.size() == 3) ? parts[2].toInt(&ok) : 0;
      if (parts.size() == 3 && (!ok || l < 1 || l > 25)) {
        QMessageBox::warning(&dialog, "Ошибка", "Номер мелкого уровня 1–25");
        return;
      }

      constexpr int F = 25;
      double offsetX = 0.5, offsetY = 0.5;
  
      if (parts.size() >= 2) {
        auto offM = snailOffset5(k);
        offsetX = (offM.first + 0.5) / 5.0;
        offsetY = (offM.second + 0.5) / 5.0;
      }
      if (parts.size() == 3) {
        auto offM = snailOffset5(k), offF = snailOffset5(l);
        offsetX = (offM.first + (offF.first + 0.5)/5.0) / 5.0;
        offsetY = (offM.second + (offF.second + 0.5)/5.0) / 5.0;
      }
  
      double x_fine = (coarseX + offsetX) * F;
      double y_fine = (coarseY + offsetY) * F;
      x_fine = std::ceil(x_fine - 1);
      y_fine = std::ceil(y_fine - 1);
  
      // шлём параметры
      if (!robot_param_client->wait_for_service(std::chrono::seconds(2))) {
        QMessageBox::warning(&dialog, "Ошибка",
                             "Сервис robot_controller недоступен");
        return;
      }
      robot_param_client->set_parameters({ rclcpp::Parameter("clear", true) });
      robot_param_client->set_parameters({ rclcpp::Parameter("clear", false) });
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      robot_param_client->set_parameters({
        rclcpp::Parameter("robot_x", x_fine),
        rclcpp::Parameter("robot_y", y_fine)
      });
      dialog.accept();
    });

    QObject::connect(buttonBox, &QDialogButtonBox::rejected,
      [&dialog]() { dialog.reject(); });    
      
    dialog.exec();
  });



  // === Кнопка "Узнать местоположение робота" ===
  QToolButton* locateButton = new QToolButton(mainWindow);
  locateButton->setText("Узнать местоположение");
  locateButton->setStyleSheet(
      "QToolButton { border: 2px solid #555; border-radius: 4px; padding: 4px; }"
      "QToolButton:hover { background-color: #ddd; }"
  );
  locateButton->setAutoRaise(false);
  toolBar->addWidget(locateButton);

  // функционал кнопки "Узнать местоположение" 
  QObject::connect(locateButton, &QToolButton::clicked, [widget, mainWindow]() {
    // 1) Включаем отображение линий
    widget->setLandmarkLinesVisible(true);
  
    // 2) Показываем модальное окно
    QDialog dialog(mainWindow);
    dialog.setWindowTitle("Местоположение робота");
    QVBoxLayout* layout = new QVBoxLayout(&dialog);
    QString infoText = QString("Координаты робота:\n x = %1\n y = %2")
    .arg(widget->estimatedX(), 0, 'f', 2)
    .arg(widget->estimatedY(), 0, 'f', 2);
    layout->addWidget(new QLabel(infoText));      
    dialog.exec();
  
    // 3) После закрытия — снова скрываем линии
    widget->setLandmarkLinesVisible(false);
  });


  
  // === Кнопка "Построить маршрут" ===
  QToolButton* routeButton = new QToolButton(mainWindow);
  routeButton->setText("Построить маршрут");
  routeButton->setStyleSheet(
      "QToolButton { border: 2px solid #555; border-radius: 4px; padding: 4px; }"
      "QToolButton:hover { background-color: #ddd; }"
  );
  toolBar->addWidget(routeButton);

  // Функционал кнопки "Построить маршрут"
  QObject::connect(routeButton, &QToolButton::clicked,
    [mainWindow, goal_param_client, makeCoordValidator]() {
      QDialog dialog(mainWindow);
      dialog.setWindowTitle("Целевые координаты");
      QVBoxLayout* layout = new QVBoxLayout(&dialog);
      QLabel* inputLabel = new QLabel(
        "Введите координаты в виде <span style='color:red'>*</span>x<span style='color:red'>*</span>y k l"
      );
      inputLabel->setTextFormat(Qt::RichText);
      layout->addWidget(inputLabel);

      // Поле ввода с тем же валидатором
      QLineEdit* inputEdit = new QLineEdit();
      inputEdit->setPlaceholderText("Например: A1 5 23");
      inputEdit->setValidator(makeCoordValidator(&dialog));
      layout->addWidget(inputEdit);

      // Описание
      QLabel* desc = new QLabel(
        "<span style='color:red'>*</span> обязательные переменные<br>"
        "x — абсцисса на карте верхнего уровня<br>"
        "y — ордината на карте верхнего уровня<br>"
        "k — номер дочерней ячейки по \"улитке\" на карте<br>"
        "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; среднего уровня (от 1 до 25)<br>"
        "l — номер дочерней ячейки по \"улитке\" на карте<br>"
        "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; нижнего уровня (от 1 до 25)"
    );
      desc->setTextFormat(Qt::RichText);
      layout->addWidget(desc);

      // Кнопки ОК/Отмена
      QDialogButtonBox* buttonBox =
        new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
      layout->addWidget(buttonBox);

      // Слот на ОК
      QObject::connect(buttonBox, &QDialogButtonBox::accepted,
        [&dialog, inputEdit, goal_param_client]() {
          QString text = inputEdit->text().trimmed();
          if (text.isEmpty()) {
            QMessageBox::warning(&dialog, "Ошибка",
                                 "Введите хотя бы координату верхнего уровня");
            return;
          }
          QStringList parts = text.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
          if (parts.size() < 1 || parts.size() > 3) {
            QMessageBox::warning(&dialog, "Ошибка",
                                 "Формат: A1, A1 k или A1 k l");
            return;
          }

          // Парсим coarse
          QString coarse = parts[0].toUpper();
          QChar letter = coarse[0];
          bool ok;
          int coarseY = coarse.mid(1).toInt(&ok);
          if (!ok) {
            QMessageBox::warning(&dialog, "Ошибка",
                                 "Неверный формат верхнего уровня");
            return;
          }
          int coarseX = letter.unicode() - QChar('A').unicode();

          // Парсим k, l
          int k = 0, l = 0;
          if (parts.size() >= 2) {
            k = parts[1].toInt(&ok);
            if (!ok || k < 1 || k > 25) {
              QMessageBox::warning(&dialog, "Ошибка",
                                   "Номер среднего уровня 1–25");
              return;
            }
          }
          if (parts.size() == 3) {
            l = parts[2].toInt(&ok);
            if (!ok || l < 1 || l > 25) {
              QMessageBox::warning(&dialog, "Ошибка",
                                   "Номер мелкого уровня 1–25");
              return;
            }
          }

          // Вычисляем x_fine, y_fine
          constexpr int F = 25;
          double offsetX = 0.5, offsetY = 0.5;
          if (k > 0) {
            auto offM = snailOffset5(k);
            offsetX = (offM.first + 0.5) / 5.0;
            offsetY = (offM.second + 0.5) / 5.0;
          }
          if (l > 0) {
            auto offM = snailOffset5(k), offF = snailOffset5(l);
            offsetX = (offM.first + (offF.first + 0.5)/5.0) / 5.0;
            offsetY = (offM.second + (offF.second + 0.5)/5.0) / 5.0;
          }
          double x_fine = (coarseX + offsetX) * F;
          double y_fine = (coarseY + offsetY) * F;
          x_fine = std::ceil(x_fine - 1);
          y_fine = std::ceil(y_fine - 1);

          // Отправляем goal_x, goal_y
          if (!goal_param_client->wait_for_service(std::chrono::seconds(2))) {
            QMessageBox::warning(&dialog, "Ошибка",
                                 "Сервис goal_handler недоступен");
            return;
          }
          goal_param_client->set_parameters({ rclcpp::Parameter("clear", true) });
          goal_param_client->set_parameters({ rclcpp::Parameter("clear", false) });
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          
          goal_param_client->set_parameters({
            rclcpp::Parameter("goal_x", x_fine),
            rclcpp::Parameter("goal_y", y_fine)
          });

          dialog.accept();
        });

      QObject::connect(buttonBox, &QDialogButtonBox::rejected,
                       [&dialog]() { dialog.reject(); });

      dialog.exec();
  });

  mainWindow->resize(width, height);
  mainWindow->show();
  return mainWindow;
}
