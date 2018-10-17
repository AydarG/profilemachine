/* программа работы станка с участием пневмомотора, на станок устанавливаются два параллельных профиля, из которых изготавливаются на параллельных
 *  ручьях два одинаковых изделия А и В.Изделия изготавливаются следующим образом. Предварительно участок профилей нагревается, затем после пере- 
 *  мещения  на необходимое расстояние профиля прокалываются, затем изделия отрезаются с правой стороны под углом 70%. Затем один из профилей (А) сдвигается на расстояние Х влево, 
 *  чтобы скомпенсировать изменение длины профиля А перед отпилом под углом -70%, затем отпиливаем оба профиля левой пилой.
 *   Для отображения количества изготовленных изделий используется жидкокристаллический дисплей LCD, который подключается по шине I2C , который подключается по пинам SDA и SDL на ардуино                                   
 *   Присваиваем пинам название переменных  - 
 */

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#define DATA_PIN 7      // пин данных (англ. data)
#define LATCH_PIN1 12   // пин строба1 (англ. latch) //  состояние узлов оборудования
#define LATCH_PIN2 10   // пин строба2 (англ. latch)  // состоятие пневмоклапанов на пневмомоторе
#define CLOCK_PIN 11    // пин такта (англ. clock)
const int PIN_STOP = 8; // светодиод можно поставить, сигнал, что пневмодвигатель остановлен
const int PIN_DRIVE = 9;
const int STATE_WAIT_START = 0;   // статусы состояний оборудования= пуск
const int STATE_CARET_VPRAVO = 1; //                      движение каретки1 вправо
const int STATE_CARET_VLEVO = 2;  //                     движение каретки1 влево
int instrumentIndex = 0;
/* в программе для экономии исползуемых пинов применена схема с использование сдвигового регистра 74HC595. Для этой цели используется отдельная функция sdvigg,
 *  которую можно применять в других программах. НА пине DATA_PIN последовательно после команды shiftOut в размере одного байта отображаются биты из массива
 *  sigments (а именно те, значение которых обозначено символом instrumentIndex ). Для начала записи пин строба LACHT_PIN притягивают к земле,
 *  для отображения на пина Qх регистра  на пин строба подают высокий сигнал
 *  Здесь- последовательно на ножках регистра появляется комбинация высоких и низких сигналов, которая для наглядности  на восьмиразрядном индикаторе 
 *  отображается в видн последовательности цифр 1,2,4,8,7,6,5 с различной продолжительностью .
 *  сигналы на 8 ми пинах отображают состояние узлов (активное, пассивное) в зависимости от текущей технологической операции 
 *   всего имеем 10 различных сочетаний взаимодействия узлов оборудования, сочетания назовем номерами операций CASE от 3 до 12, создадим массив  senments
 *  
 *                                       SAW2    CARET2           DRILL      SAW      HEAT  PRESS_CARET   PRESS_CARET2            
 *                       *                       
  */
//                                                                                                             Instrument Index
//                                       0         0       0       0       0         1        0            0           0               только нагрев
const int STATE_CASE1 = 3; //             0         0       0       0       0         0        1            0           1               Зажим профиля1,
const int STATE_CASE2 = 4; //             0         0       0       0       0         1        1            0           2               профиль1 зажат, нагрев,

const int STATE_CASE3 = 5; //            0         0       0       1       1         1        1            0           3               профиль1 зажат, нагрев, пила, перфорация

const int STATE_CASE4 = 6; //            0         0       0       1       1         0        1            0           4               профиль1 зажат,          пила, перфорация

const int STATE_CASE5 = 7; //            0         0       0       0       0         0        0            1           5               профиль1 свободен, зажим профиля2

//                                         0         1       0       0       0         0        0            1           6               профиль1 свободен,зажим профиль2, сдвиг профиля2
//                                         1         1       0       0       0         0        1            1           7               профиль1 зажат,зажим профиль2, сдвиг профиля2, пила2,
//                                         0         1       0       0       0         0        1            0           8               профиль1 зажат, ,  профиль2 свободен,

//                                         0         0       0       0       0         0        0            0           9               CARET_RELEASE, все узлы свободны 0
const int STATE_CASE6 = 8; //  анализ завершения обработки профиля
const int STATE_CASE7 = 9; //      движение до промежуточного концевика
const int STATE_MOTOR = 10;

const int INSTRUMENT_HEAT = 0;
const int INSTRUMENT_PUSH = 1;
const int INSTRUMENT_HEAT_PRESSED = 2;

byte instruments[10] = {0b00000100, 0b00000010, 0b00000110, 0b00011110, 0b00011010, 0b00000001, 0b010000001, 0b11000011, 0b01000010, 0b00000000};
//  в массиве представлены состояния команд на пневмоклапанах элементов оборудования
byte motors[10] = {0b00011000, 0b00011100, 0b00001100, 0b00001110, 0b00000110, 0b00000111, 0b00000011, 0b00010011, 0b00010001, 0b00011001};
// в массиве представлены состояния команд на пневмоклапанах мотора
byte stopMotorBits = 0b00000000;                                                                // состояние останова мотора
const int output_count = 6;                                                                     //сигналы на соответствующих пинах в количестве 6 штук                                          ИСПРАВИТЬ
int outputs[output_count] = {DATA_PIN, LATCH_PIN1, LATCH_PIN2, CLOCK_PIN, PIN_STOP, PIN_DRIVE}; //  обозначаем как выходы

const int WAIT_START = 5;                          // кнопка пуск
const int WAIT_STOP = 4;                           // отладочная кнопка для проверки работоспособности узлов, будет удалена после отладки
const int input_count = 2;                         // сигналы на соответствующих пинах в кол 4 штук
int inputs[input_count] = {WAIT_START, WAIT_STOP}; // обозначаем как входы
const int PRESS_TIME = 100;                        // время за которое происходит зажим профиля. TODO:    отладить на станке
const int SAW_TIME = 1500;                         // время отпила                                         отладить на станке
const int TIME_HEAT = 2000;                        // время нагрева
const int TIME_HEAT_ADDITIONAL = 500;              // время нагрева дополнительно
const int TIME_SAW_RETURN = 500;

int tim = 200; // задержка между командами на цилиндры пневмомотора                                                   ОТЛАДИТЬ
int tim1 = tim / 2;
int tim2 = tim / 4;
volatile int state = STATE_WAIT_START; // первоначальный статус оборудования
volatile int count = 0;                // счетчик итераций, равен 1, когда происходит только  нагрев первого изделия,равен 2, когда первое изделие
                                       //из профиля изготовлено,последнее изделие на профиле изготовлено=9, итого 8 изделий
int total = 0;                         // счетчик общего количества изделий за время работы станка
const int MAX_COUNT = 8;               // максимальное количество изделий из одного профиля
bool moveRight = true;
int motorPhase;

void setup()
{
  lcd.begin();     //инициализация LCD индикатора Подключаем 5 вольт, землю, на SDA и SDL пины на ардуино
  lcd.backlight(); // включаем подсветку
  lcd.print("QUANTITY");
  delay(1000);

  lcd.setCursor(0, 1);
  lcd.print("TOTAL");

  Serial.begin(9600);

  for (int i = 0; i < output_count; i++)
  {
    pinMode(outputs[i], OUTPUT); // обозначаем пины с соответствующими номерами как выходные
  }
  sdvig(DATA_PIN, CLOCK_PIN, LATCH_PIN1, 0);
  stopMotor();

  for (int i = 0; i < input_count; i++)
  {
    pinMode(inputs[i], INPUT_PULLUP); //обозначаем пины с соответствующими номерами как входные
  }
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  // кнопку внешнего прерывания  применить для останова мотора при перемещении вправо и лево)
  attachInterrupt(INT0, interrupt, FALLING); // подключить кнопку, по команде которой происходит прерывание к INT0 ( к пину 2) правый и левый концевик
  //кнопка внешнего прерывания применить для останова мотора при перемещении вправо до промежуточного концевика
  attachInterrupt(INT1, interruptMiddle, FALLING); // подключить кнопку, по команде которой происходит прерывание к   INT1 ( к пину3) промежуточный концевик
}

void loop()
{
  int nextState;
  switch (state)
  {                      // запуск цикла состояний работы оборудования
  case STATE_WAIT_START: // статус- ожидание запуска оборудования, кнопка пуск
    Serial.println(" ждем кнопки пуск");
    wait(WAIT_START, STATE_CARET_VPRAVO, 100); //       Запуск функции ожидания  нажатия кнопки пуск, опрос  производится через 0,1 сек,

    break;                 //             после нажатия кнопки пуск оборудование перейдет в статус - движение каретки вправо
  case STATE_CARET_VPRAVO: // статус-  движение каретки вправо
    Serial.println("МОТОР ВПРАВО");
    motorPhase = 0;
    moveRight = true;
    state = STATE_MOTOR;
    break;
  case STATE_CARET_VLEVO: // статус-  движение каретки влево
    Serial.println("МОТОР ВЛЕВО");
    motorPhase = 0;
    moveRight = false;
    state = STATE_MOTOR;
    break;
  case STATE_MOTOR:
    startMotor(motorPhase); // отображаются команды, которые поступают на пять пневмоклапанов мотора,
                                                                // включая одновременно то два, то три клапана с  переменной задержкой tim3
                                                                // в массиве sigments видно, что каждому состоянию j соответствует или 2 или 3 включенных клапана
    delay(motorPhase % 2 == 0 ? tim1 : tim2);
    motorPhase = (10 + motorPhase + (moveRight ? 1 : -1)) % 10;
    break;

  case STATE_CASE1: // статус- прижим профиля на каретке
    useInstrument(INSTRUMENT_PUSH);
    Serial.println("ПРИЖИМ");
    delay(PRESS_TIME); //         время задержки на прижим профиля в каретке
    state = STATE_CARET_VLEVO;
    break;
  case STATE_CASE2:                         // статус-включаем нагрев профиля,
    useInstrument(INSTRUMENT_HEAT_PRESSED); // нагреваем первое изделие на профиле, не пилим
    Serial.println("Подготовка первой детали профиля");
    delay(TIME_HEAT);               //  Отладить  ВРЕМЯ НАГРЕВА
    useInstrument(INSTRUMENT_HEAT); //      только нагрев
    delay(TIME_HEAT_ADDITIONAL);
    count = 1;                  // первая операция прошла -нагрели первый профиль, но не пилили
    state = STATE_CARET_VPRAVO; //    переход к статусу продвигаем Дальше
    break;
  case STATE_CASE3:   // статус-включаем нагрев профиля, опускание пилы, перфорация
    useInstrument(3); //  ОТладить   КРАЙНИМ БУДЕТ ВРЕМЯ НЕОБХОДИМОЕ НА НАГРЕВ ИЛИ ПИЛЕНИЕ ИЛИ перфорацию
    Serial.println("Основная обработка профиля");
    delay(TIME_HEAT); //  НАСТРОИТЬ
    useInstrument(1); //      пила выкл, прижим нагрева выкл, перфорация выкл, профиль ЗАЖАТ!!!
    delay(TIME_HEAT_ADDITIONAL);
    state = STATE_CASE5; //    переход к статусу зажим изделия А
    break;
  case STATE_CASE4: //  статус- включаем пилу и перфоратор
    useInstrument(4);
    Serial.println("Обработка последней детали");
    delay(SAW_TIME);
    useInstrument(1);           //     пила выкл, прижим нагрева выкл, перфорация выкл,профиль ЗАЖАТ!!!
    delay(TIME_SAW_RETURN);     //     НАСТРОИТЬ
    state = STATE_CASE5;        //     переход к  статусу- зажим изделия А
    break;
  case STATE_CASE5: //статус-зажим изделия А
    useInstrument(5);
    Serial.println("Зажим изделия А");
    delay(200);       // время отладить
    useInstrument(6); // сдвиг профиля А
    Serial.println("Сдвиг изделия А");
    delay(200);       // время  НАСТРОИТЬ
    useInstrument(7); // пила2
    Serial.println("Пила 2");
    delay(SAW_TIME);  // НАСТРОИТЬ время работы пилы2
    useInstrument(8); // профиль А свободен
    Serial.println("Профиль 2 свободен");
    delay(200);       // НАСТРОИТЬ
    useInstrument(9); // все свободно
    Serial.println("Все свободно");
    delay(PRESS_TIME);
    Serial.print("ОБРАБОТКА ДЕТАЛИ ");
    Serial.print(count + 1);
    Serial.println(" ЗАВЕРШЕНА");
    Serial.print("Total  ");
    Serial.println(total);

    count++; //               изделие изготовлено, увеличиваем счетчик
    total++; //               общее количество изделий
    // Устанавливаем курсор на вторую строку и нулевой символ.
    lcd.setCursor(11, 1);
    // Выводим на экран общее количество произведенных изделий
    lcd.print(total);
    state = STATE_CASE6;
    break;
  case STATE_CASE6: // анализ по завершения операций на профиле
    if (count == MAX_COUNT + 1)
    {                      //последняя,восьмая деталь обработана
      state = STATE_CASE7; // запускаем новый профиль
    }
    else
    {
      state = STATE_CARET_VPRAVO;
    } // все 8 деталей обрабатываются
    break;
  case STATE_CASE7: //    новый профиль пошел
    moveRight = true;
    motorPhase = 0;
    state = STATE_CARET_VPRAVO;
    break;
  }
}

// функция которая определяет следующее состояние в зависимости от номера изделия
int getNextState(int count)
{ //
  if (count == 0)
  {
    return STATE_CASE2;
  }
  if (count < MAX_COUNT + 1)
  {
    return STATE_CASE3;
  }
  else
  {
    return STATE_CASE4;
  }
}

// функция ожидания замыкания и переход на следующее состояние
void wait(int input, int toState, int delayMs)
{
  bool value = digitalRead(input); // опрашиваем пин, указанный в скобках
  // при замыкании 
  if (!value)
  {                   
    state = toState;
  }
  delay(delayMs);
}

void sdvig(int output1, int output2, int output3, byte bits)
{
  // для записи в 74HC595 нужно притянуть пин строба к земле
  digitalWrite(output3, LOW);
  // задвигаем (англ. shift out) байт-маску бит за битом,
  // начиная с младшего (англ. Least Significant Bit first)
  shiftOut(output1, output2, LSBFIRST, bits);
  // чтобы переданный байт отразился на выходах Qx, нужно
  // подать на пин строба высокий сигнал
  digitalWrite(output3, HIGH);
}

void useInstrument(int instrumentIndex)
{
  Serial.print("Используем инстументы в комбинации: ");
  Serial.println(instrumentIndex);
  sdvig(DATA_PIN, CLOCK_PIN, LATCH_PIN1, instrumentIndex);
}

void startMotor(int motorPhase){
  sdvig(DATA_PIN, CLOCK_PIN, LATCH_PIN2, motors[motorPhase]);
  digitalWrite(PIN_DRIVE, HIGH);
  digitalWrite(PIN_STOP, LOW);
}

void stopMotor()
{
  sdvig(DATA_PIN, CLOCK_PIN, LATCH_PIN2, stopMotorBits);
  digitalWrite(PIN_DRIVE, LOW);
  digitalWrite(PIN_STOP, HIGH);
}

void interrupt()
{
  if (state != STATE_MOTOR)
    return;
  Serial.print("Концевик нажат ");

  stopMotor();
  if (moveRight)
  {
    Serial.println("ВПРАВО");
    state = STATE_CASE1;
  }
  else
  {
    Serial.println("ВЛЕВО");
    state = getNextState(count);
  }
}

void interruptMiddle()
{
  if (state != STATE_MOTOR)
    return;
  if (count == 9)
  {
    stopMotor();
    count = 0;
    state = STATE_CASE1;
  }
}
//отладочная  функция ожидания замыкания кнопки СТОП, будет удалена после отладки программы
