#include <Ticker.h>
#include <AsyncTCP_RP2040W.h>
#include <AsyncUDP_RP2040W.h>
#include <vector>

//Управляющие линии задних колес
#define PIN_BACKLEFT_F      7
#define PIN_BACKLEFT_R      6
#define PIN_BACKRIGHT_F     28
#define PIN_BACKRIGHT_R     27
//Управляющие линии передних колес
#define PIN_FORWARDRIGHT_F  9
#define PIN_FORWARDRIGHT_R  8
#define PIN_FORWARDLEFT_F   20
#define PIN_FORWARDLEFT_R   21

//Сигнальные линии энкодера заднего левого колеса
#define PIN_ENC_BACKLEFT_A  3
#define PIN_ENC_BACKLEFT_B  2
//Сигнальные линии энкодера заднего правого колеса
#define PIN_ENC_BACKRIGHT_A  5
#define PIN_ENC_BACKRIGHT_B  4
//Сигнальные линии энкодера переднего левого колеса
#define PIN_ENC_FORWARDLEFT_A  13
#define PIN_ENC_FORWARDLEFT_B  12
//Сигнальные линии энкодера заднего правого колеса
#define PIN_ENC_FORWARDRIGHT_A  15
#define PIN_ENC_FORWARDRIGHT_B  14

//TCP-порт для одиночных команд управления вектором движения Omnibot
#define PORT          5698
//Размер буфера приема по TCP-порту
#define REPLY_SIZE      64

double r = 0.04;        //Радиус Mecanum-колеса,
double lx = 0.055;      //Расстояние от продольной оси шасси до продольной линии опоры колес, м
double ly = 0.1;        //Расстояние от поперечной оси шасси до линии осей колес, м

static std::vector<AsyncClient*> clients; //Список клиентов TCP-сервера
char ssid[] = "RoboLab";      //Идентификатор WiFi-сети робота (в режиме: "точка доступа" - AP)
//char ssid[] = "Omnibot";      //Идентификатор WiFi-сети робота (в режиме: "точка доступа" - AP)
char pass[] = "Qwe123!!";     //Пароль доступа к WiFi-сети
int status = WL_IDLE_STATUS;  //Инициализация статуса соединения
IPAddress serverIP;           //IP-адрес Оmnibot (сервера)

Ticker timerPID;              //Таймер ПИД-регулятора

//Класс управления мотором постоянного тока (JGB37-520)
class Motor{
  public:
    Motor(uint8_t pin_Forward, uint8_t pin_Reverse, uint8_t pin_EncA, uint8_t pin_EncB, bool isRightMotor);
    void setEncoderISR(void (*ISR_EncA)(void), void (*ISR_EncB)(void));
    void ISR_EncA();
    void ISR_EncB();
    void setPower(int16_t power);
    double getOmega();
    void setOmega(double omega);
    double getAngle();
    void ControlOmega();
    void setPosition(double pos);
    void Action(double dtime, double startPos, double omega);
  private:
    bool isRight;
    uint8_t lineForward;
    uint8_t lineReverse;
    uint8_t lineEncA;
    uint8_t lineEncB;
    uint32_t timeA, timeB;
    int32_t dt;
    int32_t countTickEnc;
    double stepAngle;
    double desiredOmega;
    uint32_t t0;
    double angle0, prevAngle;
    double Kp_omega, Ki_omega, Kp_angle, Kd_angle;
};

Motor::Motor(uint8_t pin_Forward, uint8_t pin_Reverse, uint8_t pin_EncA, uint8_t pin_EncB, bool isRightMotor){
  //Настройка и инициализацияя линий управления мотором
  lineForward=pin_Forward;
  lineReverse=pin_Reverse;
  pinMode(lineForward,OUTPUT);
  pinMode(lineReverse,OUTPUT);
  digitalWrite(lineForward, LOW);
  digitalWrite(lineReverse, LOW);
  //Настройка и инициализация линий контроля квадратурного энкодера
  lineEncA=pin_EncA;
  lineEncB=pin_EncB;
  pinMode(lineEncA,INPUT_PULLUP);
  pinMode(lineEncB,INPUT_PULLUP);
  //Инициализация параметров мотора
  isRight=isRightMotor;     //Признак правого мотора: для правых (перевернутых) моторов должен меняться знак желаемой скорости
  dt=0x7fffffff;            //Временной интервал между прерываниями энкодера: инициализация максимально возможным значением
  timeA=timeB=micros();     //Время крайнего прерывания на каналах (А и В) энкодера
  countTickEnc=0;           //Количество шагов (stepAngle) в повороте колеса
  stepAngle=PI/1850;        //Шаг угла поворота колеса между прерываниями на одном канале квадратурного энкодера
  Kp_omega=200;             //Коэффициент пропорционального регулятора угловой скорости
  Ki_omega=1000;            //Коэффициент интегрального регулятора угловой скорости
  Kp_angle=4000;            //Коэффициент пропорционального регулятора угла
  Kd_angle=1e5;             //Коэффициент дифференциального регулятора угла
  desiredOmega=0;           //Желаемое значение угловой скорости колеса в радианах
  t0=timeA;                 //Время в момент изменения желаемой скорости
  angle0=prevAngle=0;       //Угол поворота оси в момент измененияя желаемой скорости                 
}

void Motor::ISR_EncA(){
  long t=micros();
  unsigned char levelA = digitalRead(lineEncA);
  unsigned char levelB = digitalRead(lineEncB);
  dt=timeA-t;
  timeA=t;
  if(levelA==!isRight){
    if(levelB==HIGH) dt=-dt;
  }else{
    if(levelB==LOW) dt=-dt;
  }
  if(dt>0)
    countTickEnc++;
  else
    countTickEnc--;
  //Serial.println(countTickEnc);
}

void Motor::ISR_EncB(){
  long t=micros();
  unsigned char levelA = digitalRead(lineEncA);
  unsigned char levelB = digitalRead(lineEncB);
  dt=timeB-t;
  timeB=t;
  if(levelB==!isRight){
    if(levelA==LOW) dt=-dt;
  }else{
    if(levelA==HIGH) dt=-dt;
  }
}

void Motor::setEncoderISR(void (*ISR_EncA)(void), void (*ISR_EncB)(void)){
  attachInterrupt(digitalPinToInterrupt(lineEncA),ISR_EncA,CHANGE);
  attachInterrupt(digitalPinToInterrupt(lineEncB),ISR_EncB,CHANGE);
}

double Motor::getOmega(){
  uint32_t t=micros();
  uint32_t dt3=3*abs(dt);
  if((t-timeA>dt3)||(t-timeB>dt3)) 
    return 0;
  else
    return stepAngle*1e6/dt;
}

void Motor::setOmega(double omega){
    t0=micros();
    angle0=countTickEnc*stepAngle;
    desiredOmega=omega;
}

void Motor::setPosition(double pos){
  countTickEnc = uint32_t(pos / stepAngle);
  setOmega(0);
}

void Motor::Action(double dtime, double startPos, double omega){
    t0=micros();
    angle0=startPos;
    desiredOmega=omega;
    delayMicroseconds(dtime);
}

void Motor::setPower(int16_t power){
  if(power>255) power=255;
  if(power<-255) power=-255;
  if(power>0){
    analogWrite(lineReverse,0);
    analogWrite(lineForward,power);
  }else{
    analogWrite(lineForward,0);
    analogWrite(lineReverse,abs(power));
  }
}

double Motor::getAngle(){
  return countTickEnc*stepAngle;
}

void Motor::ControlOmega(){
  double t=(micros()-t0)/1e6;
  double angle = getAngle();
  if (desiredOmega!=0) {
    setPower(Kp_omega*(desiredOmega-getOmega())+Ki_omega*(angle0+desiredOmega*t-angle)); //ПИ-регулятор угловой скорости
  } else {
    setPower(Kp_angle*(angle0-angle)+Kd_angle*(prevAngle-angle));   //ПД-регулятор углового положения 
  }
  prevAngle=angle;
}
//================================================================================================================


//Создание объекта управления задним левым колесом и глобальных обработчиков прерываний его энкодера==============
Motor BackLeftMotor(PIN_BACKLEFT_F,PIN_BACKLEFT_R,PIN_ENC_BACKLEFT_A,PIN_ENC_BACKLEFT_B, false);
//Глобальная процедура - обработчик прерывания для канала А энкодера заднего левого колеса
void ISR_BackLeftEncA(){
  BackLeftMotor.ISR_EncA();
}
//Глобальная процедура - обработчик прерывания для канала B энкодера заднего левого колеса
void ISR_BackLeftEncB(){
  BackLeftMotor.ISR_EncB();
}
//================================================================================================================

//Создание объекта управления задним правым колесом и глобальных обработчиков прерываний его энкодера=============
Motor BackRightMotor(PIN_BACKRIGHT_F,PIN_BACKRIGHT_R,PIN_ENC_BACKRIGHT_A,PIN_ENC_BACKRIGHT_B, true);
//Глобальная процедура - обработчик прерывания для канала А энкодера заднего правого колеса
void ISR_BackRightEncA(){
  BackRightMotor.ISR_EncA();
}
//Глобальная процедура - обработчик прерывания для канала B энкодера заднего правого колеса
void ISR_BackRightEncB(){
  BackRightMotor.ISR_EncB();
}
//================================================================================================================

//Создание объекта управления передним левым колесом и глобальных обработчиков прерываний его энкодера============
Motor ForwardLeftMotor(PIN_FORWARDLEFT_F,PIN_FORWARDLEFT_R,PIN_ENC_FORWARDLEFT_A,PIN_ENC_FORWARDLEFT_B, false);
//Глобальная процедура - обработчик прерывания для канала А энкодера переднего левого колеса
void ISR_ForwardLeftEncA(){
  ForwardLeftMotor.ISR_EncA();
}
//Глобальная процедура - обработчик прерывания для канала B энкодера переднего левого колеса
void ISR_ForwardLeftEncB(){
  ForwardLeftMotor.ISR_EncB();
}
//=================================================================================================================


//Создание объекта управления передним правым колесом и глобальных обработчиков прерываний его энкодера============
Motor ForwardRightMotor(PIN_FORWARDRIGHT_F,PIN_FORWARDRIGHT_R,PIN_ENC_FORWARDRIGHT_A,PIN_ENC_FORWARDRIGHT_B, true);
//Глобальная процедура - обработчик прерывания для канала А энкодера переднего правого колеса
void ISR_ForwardRightEncA(){
  ForwardRightMotor.ISR_EncA();
}
//Глобальная процедура - обработчик прерывания для канала B энкодера переднего правого колеса
void ISR_ForwardRightEncB(){
  ForwardRightMotor.ISR_EncB();
}
//==================================================================================================================


//Глобальная процедура таймера для обеспечения работы PID-регуляторов всех колес====================================
void timer_ISR(){
  BackLeftMotor.ControlOmega();
  BackRightMotor.ControlOmega();
  ForwardLeftMotor.ControlOmega();
  ForwardRightMotor.ControlOmega();
}
//===================================================================================================================

//Callback-процедуры обработки событий взаимодействия с сетевым клиентом=============================================

//Обработчик ошибок соединения с сетевым клиентом
static void handleError(void* arg, AsyncClient* client, int8_t error)
{
  (void) arg;
  Serial.printf("\nОшибка соединения %s с клиентом %s \n", client->errorToString(error), client->remoteIP().toString().c_str());
}

//Обработчик приема данных от сетевого клиента
static void handleData(void* arg, AsyncClient* client, void *data, size_t len)
{
  (void) arg;
  //Прием данных от клиента и вывод их в мониторинговый порт
  Serial.printf("\nОт клиента получены данные: %s \n", client->remoteIP().toString().c_str());
  Serial.write((uint8_t*)data, len);
  //Анализ (данных) командной строки
  char delimiter[] = ";";
  char *value = strtok((char*)data, delimiter);
  double Speed=atof(value);
  Serial.printf("\nСкорость движения, м/с: %.2f%\n",Speed);
  value = strtok(NULL, delimiter);
  double Alpha=atof(value);
  Serial.printf("Направление движения, рад: %.2f%\n",Alpha);
  value = strtok(NULL, delimiter);
  double Omega=atof(value);
  Serial.printf("Угловая скорость шасси, рад/с: %.2f%\n",Omega);
  if(Speed==0 && Omega==0){
    BackLeftMotor.setOmega(0); 
    BackRightMotor.setOmega(0);
    ForwardLeftMotor.setOmega(0);
    ForwardRightMotor.setOmega(0);
    delay(500);
    BackLeftMotor.setPosition(BackLeftMotor.getAngle()); 
    BackRightMotor.setPosition(BackRightMotor.getAngle());
    ForwardLeftMotor.setPosition(ForwardLeftMotor.getAngle());
    ForwardRightMotor.setPosition(ForwardRightMotor.getAngle()); 
  } else {
  //Обработка данных: расчет угловых скоростей колес робота
    double Vx = Speed * cos(Alpha);
    double Vy = Speed * sin(Alpha);
    double lfW=(Vx-Vy-(lx+ly)*Omega)/r;
    double rfW=(Vx+Vy+(lx+ly)*Omega)/r;
    double lbW=(Vx+Vy-(lx+ly)*Omega)/r;
    double rbW=(Vx-Vy+(lx+ly)*Omega)/r;
    //Настройка ПИД-регуляторов колес 
    BackLeftMotor.setOmega(lbW); 
    BackRightMotor.setOmega(rbW);
    ForwardLeftMotor.setOmega(lfW);
    ForwardRightMotor.setOmega(rfW);
  }
  //Ответ клиенту
  if (client->space() > REPLY_SIZE && client->canSend())
  {
    char reply[REPLY_SIZE];
    sprintf(reply, "Запрос принят Omnibot @ %s", serverIP.toString().c_str());
    client->add(reply, strlen(reply));
    client->send();
  }
}

//Обработчик события отключения сетевого клиента
static void handleDisconnect(void* arg, AsyncClient* client)
{
  (void) arg;
  Serial.printf("Соединение закрыто\n");
}

//Обработчик события таймаута при установлении соединения с сетевым клиентом
static void handleTimeOut(void* arg, AsyncClient* client, uint32_t time)
{
  (void) arg;
  (void) time;
  Serial.printf("\nACK-таймаут клиента с ip: %s\n", client->remoteIP().toString().c_str());
}

//---------------------------------------------------------------------

//Процедута подключения нового клиента---------------------------------
static void handleNewClient(void* arg, AsyncClient* client)
{
  (void) arg;
  Serial.printf("\nНовый клиент подключен к серверу. IP-клиента: %s", client->remoteIP().toString().c_str());
  //Добавление в список клиентов
  clients.push_back(client);
  //Регистрация обработчиков событий
  client->onData(&handleData, NULL);
  client->onError(&handleError, NULL);
  client->onDisconnect(&handleDisconnect, NULL);
  client->onTimeout(&handleTimeOut, NULL);
}
//---------------------------------------------------------------------

//Процедура инициализации WiFi-сети
void Init_Wifi(){
  //Информация об архитектуре Omnibot-сервера в мониторинговый порт
  Serial.print("\nИнициализация WiFi-модуля на микроконтроллере ");
  Serial.print(BOARD_NAME);
  Serial.print(" с ");
  Serial.println(SHIELD_TYPE);
  Serial.println(ASYNCTCP_RP2040W_VERSION);
  //Проверка наличия WiFi-модуля
  if (WiFi.status() == WL_NO_MODULE)
  {
    Serial.println("Коммуникации с WiFi-модулем невозможны!");
    //Зацикливание и остановка загрузки
    while (true);
  }
  //Передача информации о статусе соединения в мониторинговый порт
  Serial.print(F("Инициализация соединения с SSID: "));
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);                 //Включение WiFi-модуля Omnibot в режиме "точка доступа"
  status = WiFi.begin(ssid, pass);    //Инициализации WiFi-модуля
  delay(1000);
  //Проверка статуса соединения с SSID
  while (status != WL_CONNECTED)
  {
    delay(500);
    status = WiFi.status(); //Запрос статуса соединения
  }
  //Отображение сетевых параметров Omnibot
  Serial.print("Установлено соединение с SSID: ");
  Serial.println(WiFi.SSID());
  //Отображение сетевого адреса 
  serverIP = WiFi.localIP();
  Serial.print("Локальный IP-адрес: ");
  Serial.println(serverIP);
}

void TCP_Init(AsyncServer* server)
{
  server->onClient(&handleNewClient, server);       //Регистрация процедура обработки клиентских запросов
  server->begin();                                  //Запуск сервера

  Serial.print(F("Командный Omnibot-сервер имеет сокет: "));
  Serial.print(serverIP);
  Serial.print(F(":"));
  Serial.println(PORT);

  pinMode(LED_BUILTIN, OUTPUT);
}

void UDP_Init(AsyncUDP UDP)
{
  if (UDP.listen(PORT)) {
    Serial.printf("Now listening on UDP port %d\n", PORT);
    UDP.onPacket([](AsyncUDPPacket packet) {
      Serial.printf("Packet received! Type: %s, From: %s:%d, Length: %d, Data: %s\n",
                    packet.isBroadcast() ? "Broadcast" : packet.isMulticast() ? "Multicast" : "Unicast",
                    packet.remoteIP().toString().c_str(),
                    packet.remotePort(),
                    packet.length(),
                    packet.data());

      // Send a reply back to the sender
      packet.printf("ESP32 received your %d bytes", packet.length());
    });
  }
}

//---------------------------------------------------------------------

//Процедура инициализация, выполняемая на основном (нулевом) ядре микроконтроллера RP2040========================
void setup() {
  Serial.begin(2000000);
  //Назначение энкодеру заднего левого колеса глобальных callback-функций
  BackLeftMotor.setEncoderISR(ISR_BackLeftEncA,ISR_BackLeftEncB);
  //Назначение энкодеру заднего правого колеса глобальных callback-функций
  BackRightMotor.setEncoderISR(ISR_BackRightEncA,ISR_BackRightEncB);
  //Назначение энкодеру переднего левого колеса глобальных callback-функций
  ForwardLeftMotor.setEncoderISR(ISR_ForwardLeftEncA,ISR_ForwardLeftEncB);
  //Назначение энкодеру переднего правого колеса глобальных callback-функций
  ForwardRightMotor.setEncoderISR(ISR_ForwardRightEncA,ISR_ForwardRightEncB);
  //Запуск таймера с периодом 1 мс для регулярного контроля скорости колес 
  timerPID.attach(0.001,timer_ISR);

  Init_Wifi();  //Инициализация WiFi
}
//Циклическая процедура, выполняемая на нулевом ядре микроконтроллера
void loop() {

}

//Процедура инициализация, выполняемая на дополнительном (первом) ядре микроконтроллера RP2040=====================
void setup1(){
  delay(5000);
  //Создание объекта TCP-сервера
  AsyncServer* TCP_server = new AsyncServer(PORT);
  AsyncUDP UDP;
  // TCP_Init(TCP_server);  //Запуск сервера по порту 5698
  // UDP_Init(UDP);
}

//Циклическая процедура, выполняемая на первом ядре микроконтроллера
void loop1(){
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(500);                      // wait for a second
}