#pragma once

#ifndef TUYA_TERMO_H
#define TUYA_TERMO_H

// Скачать оригинальную прошиву: esptool -p com4 read_flash 0x00000 0x100000 fitmware1M.bin
// Залить прошивку : esptool -p com4 write_flash -fs 1MB 0x0 fitmware1M.bin
// 

#include <Arduino.h>
#include <stdarg.h>
#include <vector>
#include <string>
#include <ctime>
#include "esphome.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/number/number.h"
#include "esphome/components/lock/lock.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"


// раскоментируй ключ HOLMS для вывода лога под Эксель, значение ключа - размер пакетов которые будут видны
//#define HOLMS 19

namespace esphome {
namespace tuya_termo {

using namespace esphome;
using namespace esphome::switch_;
using namespace esphome::binary_sensor;
using namespace esphome::text_sensor;
using namespace esphome::sensor;
using namespace esphome::select;
using namespace esphome::number;
using namespace esphome::select;
using namespace esphome::lock;

static const char *const TAG = "TuyaTermo";

using climate::ClimateMode;
using climate::ClimatePreset;
using climate::ClimateTraits;
using sensor::Sensor;    
using select::Select;  
using lock::Lock;  
using text_sensor::TextSensor;    
using binary_sensor::BinarySensor;  
using time::RealTimeClock;
using uart::UARTDevice;  
using uart::UARTComponent;

// типы запросов
enum tCommand:uint8_t {PING=0,   //- периодический PING 
                       IDENT=1,  //- запрос идентификатора устройства
                       STAT=2,   //- запрос статуса термостата (занят и т.д.)
                       REPRES=4, //- ответ на ресет
                       DATA=8    //- запрос настроек термостата (расписание, режимы ECO, MANUAL, LOCK и пр.) 
}; 

// типы команд
enum tMode:uint16_t {POWER=0x0101,  //- включение/выключение
                    MANUAL=0x0404,  //- режим
                    ECO=0x0501,     //- эко
                    LOCK=0x0601     //- чилдрен лок
};

// булевы переменные протокола
enum tStat:uint8_t {OFF=0, ON=1, UNDEF=0xFF};

// типы инфы о состоянии связи
enum tMainState:uint8_t {wsOFF=1,   //- устройство не имеет настроек WIFI
                         wsPROC=2,  //- устройство имеет настройки wifi, но не подключено
                         wsOK=3,    //- успешно подключено и нужен первый набор данных
                         wsOK_4=4,  //- нужен второй набор данных
                         wsOK_5=5
};

class TuyaTermo;

//////////////////////////////////  структуры данных для работы с термостатом ///////////////////////////

// структура одной записи периода режима АВТО
struct termoSet{
   uint8_t minutes=0;
   uint8_t hours=0;
   uint8_t _temp=20;
   float temp(){
      return ((float)_temp)/2;  
   }
   void temp(float t){
      _temp=(uint8_t)(t*2);
   }
   void print(String& str){ // НЕ БЕЗОПАСНО !!!!
      char buff[16];
      sprintf(buff,"%u:%0u %f°С", hours, minutes, temp()); 
      str+=String(buff);      
   }
};

//структура работы в режиме AUTO по рассписанию
struct sPeriods{
   termoSet d[18]; // 6+6+6 записей, первые 6- рабочие дни, вторые 6 - по субботам, последние 6 по воскресениям
   void print6(termoSet* t, String& str){
      char buff[10];
      for(uint8_t i=0;i<6;i++){
        sprintf(buff, "%u: ",i);
        str+=String(buff);
        t->print(str);
        str+="; ";
        t++;   
      }
   }
   void print(String& str){
      str+="  WORK: "; print6(d, str); str+="\n";
      str+="   SAT: "; print6(d+6, str); str+="\n";
      str+="   SUN: "; print6(d+12, str); str+="\n";
   }
   // получение температуры из плана в соотвествии с временем и днем
   uint8_t get_plan_current_temp_raw(uint8_t day_of_week, uint8_t hours, uint8_t minutes){
      ESP_LOGV(TAG,"Find current period day of week: %u, hour: %u, min: %u", day_of_week,  hours,  minutes);
      uint8_t temp=0;  //стартовая температура поиска
      uint8_t start=0; //точка старта поиска
      if(day_of_week==1){// понедельник
         temp=17; //для понедельника стартовая температура последняя в воскресение
         start=0;
      } else if(day_of_week==6){//суббота
         temp=5; //для субботы стартовая температура последняя в рабочий день
         start=6;
      } else if(day_of_week==7){//воскресенье
         temp=11; //для воскресение стартовая температура последняя в субботу день
         start=12;
      } else { // для остальных (рабочих дней)
         temp=5; //для рабочих не понедельников стартовая температура последняя в рабочий день
         start=0;
      }   
      ESP_LOGV(TAG,"Params run: temp=%u, start=%u", temp, start);
      temp=d[temp]._temp; //загрузили стартовую температуру
      uint16_t curr_min=(uint16_t)60*hours+minutes; //поисковое время в минутах
      for(uint8_t i=start; i<start+6; i++){ //ищем температуру в соответствующий промежуток времени
         uint16_t period_min=(uint16_t)60*d[i].hours+d[i].minutes; //время периода в минутах
         ESP_LOGV(TAG,"Iter: %u, curr_min: %u, period_min: %u,  HOUR:%u MIN:%u", i, curr_min, period_min, d[i].hours, d[i].minutes);
         if(curr_min>=period_min){
            temp=d[i]._temp;
            ESP_LOGV(TAG,"Get next raw temperature: %u", temp);
         } else {
            break;
         }
      }
      return temp;
   }
};
 
 
//////////////////////////// наши контролы ////////////////////////////////////////////////
class TuyaTermo_Switch : public switch_::Switch, public Component, public esphome::Parented<TuyaTermo> {
 protected:
   void write_state(bool state) override { 
      if(this->state!=state){
         this->state_callback_.call(state);
         this->publish_state(state); 
      }
    }
 friend class TuyaTermo;   
};

class TuyaTermo_Number : public number::Number, public Component, public esphome::Parented<TuyaTermo> {
 protected:
    void control(float value) override {
       if(this->state!=value){
         this->publish_state(value); 
       }
    }
 friend class TuyaTermo;   
};

class TuyaTermo_Select : public select::Select, public Component, public esphome::Parented<TuyaTermo> {
 protected:
    void control(const std::string &value) override {
       if(this->state!=value){
         this->publish_state(value); 
         //this->state_callback_.call(value);
       }
    }
 friend class TuyaTermo;   
};

class TuyaTermo_Lock : public lock::Lock, public Component, public esphome::Parented<TuyaTermo> {
 protected:
    //void open_latch() {
       //... 
    //};
    void control(const LockCall &call) override {
       ESP_LOGV(TAG,"LOCK CONTROL !!!");   
    }
 friend class TuyaTermo;   
};

/////////////////// ОСНОВНОЙ ОБЪЕКТ ////////////////////////////////////
class TuyaTermo : public esphome::Component, public esphome::climate::Climate {
   private:
    
    const std::string TERMO_FIRMWARE_VERSION = "0.0.1";
    // хидер протокола обмена с термостатом
    const uint8_t COMMAND_START[2] = {0x55, 0xAA};
    const uint32_t HEARTBEAT_INTERVAL = 15; // переодичность передачи сигнала активности процессору термостата (Sec)
    const uint32_t UART_TIMEOUT = 250; //1000; !!!НОВАЯ ВЕРСИЯ!!! время ожидания uart (uS), пока это время не истечет после приема или отправки - ждем
    const uint32_t SET_PLAN_TIMEOUT = 5;    // время задержки отправки расписания в (Sec), после изменения в интерфейсе
    const uint32_t PROTO_RESTART_UNTERVAL = 20; // переодичность перезапуска протокола обмена с термостатом (Min)

    // для Select
    const std::string w1="Week day 1";
    const std::string w2="Week day 2";
    const std::string w3="Week day 3";
    const std::string w4="Week day 4";
    const std::string w5="Week day 5";
    const std::string w6="Week day 6";
    const std::string sa1="Saturday 1";
    const std::string sa2="Saturday 2";
    const std::string sa3="Saturday 3";
    const std::string sa4="Saturday 4";
    const std::string sa5="Saturday 5";
    const std::string sa6="Saturday 6";
    const std::string su1="Sunday 1";
    const std::string su2="Sunday 2";
    const std::string su3="Sunday 3";
    const std::string su4="Sunday 4";
    const std::string su5="Sunday 5";
    const std::string su6="Sunday 6";
    std::vector<std::string> str_plan={w1,   w2,  w3,  w4,  w5,  w6,
                                       sa1, sa2, sa3, sa4, sa5, sa6,
                                       su1, su2, su3, su4, su5, su6};
    // режим ускоренного обновления 
    bool _optimistic = true;
    // поддерживаемые кондиционером опции
    std::set<ClimateMode> _supported_modes{climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_AUTO};
    std::set<ClimatePreset> _supported_presets{climate::CLIMATE_PRESET_ECO};
    // Шаблон параметров отображения виджета
    esphome::climate::ClimateTraits _traits;
    // доп настройки для отработки логики термостата
    float temperature_eco=20;
    float temperature_overheat=45;
    float temperature_deadzone=1;
    // сенсоры 
    esphome::sensor::Sensor *sensor_external_temperature_{nullptr}; // выносной сенсор температуры
    esphome::sensor::Sensor *sensor_internal_temperature_{nullptr}; // внутренний сенсор температуры
    esphome::text_sensor::TextSensor *sensor_mcu_id_{nullptr}; // MCU продукт ID
    // свитч режима чилдрен лок
    TuyaTermo_Switch* lock_switch{nullptr};
    // буферы передачи состояний термостату
    tStat send_lock=UNDEF; // состояние LOCK, если 0xFF, то не изменилось
    tStat send_eco=UNDEF; // состояние ECO, если 0xFF, то не изменилось
    tStat send_manual=UNDEF; // состояние AUTO, если 0xFF, то не изменилось
    tStat send_on=UNDEF; // состояние режима ON/OFF изменилось
    uint8_t new_target_temp_raw=0xFF; //целевая температура, полученная от виджета, если не 0xFF - нужно отдать термостату
    // буферы сохраненных состояний
    uint8_t _on=0xFF;
    uint8_t manualMode=0xFF;
    uint8_t ecoMode=0xFF; 
    uint8_t unknowMode_67=0xFF;
    uint8_t unknowMode_68=0xFF;
    // для работы с временем
    esphome::time::RealTimeClock* time_{nullptr};
    // данные о состояниях
    bool set_change=false; // флаг получения новых данных от виджета
    uint8_t curr_ext_temp_raw=0xFF; // текущая температура внешняя
    uint8_t curr_int_temp_raw=0xFF; // текущая температура внутреняя
    uint8_t target_temp_raw=0xFF; // текущая целевая температура
    // указатель на UART, по которому общаемся с кондиционером
    esphome::uart::UARTComponent *_tuya_serial;
    uint8_t receivedCommand[254]; // буфер чтения
    uint8_t commandLength=0; // длинна команды определяется во время получения пакета
    int16_t receiveIndex=0; // количество принятых байт
    uint8_t sendCounter=0; //маршрутизатор отправки
    uint8_t waitReply=0xFF; //буфер крайней отправленной команды, для контроля ответа
    uint32_t lastSend=0; // время отправки последнего байта
    uint32_t lastRead=0; // время получения последнего байта
    bool sendRight=false; //флаг разрешения внеочередной отправки данных, когда идентификатор полученного ответа от терстата совпал с запросом
 
    // numbers для установки расписания
    TuyaTermo_Number* hours_number=nullptr; 
    TuyaTermo_Number* minutes_number=nullptr;
    TuyaTermo_Number* termo_number=nullptr;
    // selector для установки расписания
    TuyaTermo_Select* plan_select=nullptr; 
    // чилдрен лок
    //TuyaTermo_Lock* child_lock=nullptr;
    // нога контроля сигнала ресет протокол от MCU
    GPIOPin* in_reset_pin{nullptr};
    uint8_t  in_reset_pin_state=0xAA;
    uint32_t reset_timer=0; // таймер ресета, задержка после получения сигнала 

    // для обслуживания изменеия расписания
    sPeriods plan; // план работы в режиме авто
    uint32_t timer_plan_change=0; //флаг-таймер изменения плана работы из espHome
    uint8_t current_select_pos=0xFF; // текущий выбор в селекте
    bool plan_staff=false; // флаг поднимается во время переключения в селекте, для блокировки изменения данных
    // температура из данных в протоколе
    float getTemp(uint8_t raw){
       return float(raw)/2;
    }

    void _debugMsg(const String &msg, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0, ...) {
        if (dbgLevel < ESPHOME_LOG_LEVEL_NONE) dbgLevel = ESPHOME_LOG_LEVEL_NONE;
        if (dbgLevel > ESPHOME_LOG_LEVEL_VERY_VERBOSE) dbgLevel = ESPHOME_LOG_LEVEL_VERY_VERBOSE;

        if (line == 0) line = __LINE__;  // если строка не передана, берем текущую строку

        va_list vl;
        va_start(vl, line);
        esp_log_vprintf_(dbgLevel, TAG, line, msg.c_str(), vl);
        va_end(vl);
    }

    void _debugPrintPacket(uint8_t *packet, uint8_t length, bool dirrect=true, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0) {
        String st = "";
        char textBuf[11];

        // заполняем время получения пакета
        memset(textBuf, 0, 11);
        sprintf(textBuf, "%010u", esphome::millis());
        st = st + textBuf + ": ";

        // формируем преамбулы
        if (dirrect) {
            st += "Out: ";  // преамбула входящего пакета
        } else  {
            st += " In: ";  // преамбула исходящего пакета
        }

// формируем данные
#ifdef HOLMS
        // если этот дефайн объявлен, то в лог попадут только пакеты больше указанного в дефайне размера
        // при этом весь вывод будет в десятичном виде, данные будут разделены ";"
        // и не будет выделения заголовков и CRC квадратными скобками
        dbgLevel = ESPHOME_LOG_LEVEL_ERROR;
        for (size_t i = 0; i < length; i++) {
            memset(textBuf, 0, 11);
            sprintf(textBuf, "%02X", packet[i]);
            st += textBuf;
            if(i<length-1){
                st += ";";
            }
        }
#else
        // если дефайна HOLMS нет, то выводим пакеты в HEX и все подряд
        for (size_t i = 0; i < length; i++) {
            // для нормальных пакетов надо заключить заголовок в []
            if (i == 0) st += "[";
            // для нормальных пакетов надо заключить CRC в []
            if (i == length-1) st += "[";

            memset(textBuf, 0, 11);
            sprintf(textBuf, "%02X", packet[i]);
            st += textBuf;

            // для нормальных пакетов надо заключить заголовок в []
            if (i==5) st += "]";
            // для нормальных пакетов надо заключить CRC в []
            if (i == length-1) st += "]";

            st += " ";
        }
#endif
        if (line == 0) line = __LINE__;
        _debugMsg(st, dbgLevel, line);
    }

    // публикация текущего режима работы климата
    void check_action(){
        ESP_LOGV(TAG, "Current state publicate - not released");
    }

    // получить текст из буфера данных
    String getStringFromBuff(uint8_t* buff, uint8_t count){
       String text="";
       for(uint8_t i=0; i<count; i++){
          text+=(char)buff[i];
       }
       return text;
    }

    // очистка буфер приема
    void resetAll() {
       receiveIndex = -1;
       commandLength = -1;
    }

    // установка данных в контролы
    void refresh_controls(unsigned int pos){
       plan_staff=true; // отключаем обработки в момент загрузки данных в контролы
       {
          auto call = hours_number->make_call(); // грузим данные в контрол часов
          call.set_value((float)(plan.d[pos].hours));
          call.perform();
          hours_number->publish_state((float)(plan.d[pos].hours));
       }
       {
          auto call = minutes_number->make_call(); // грузим данные в контрол минут
          call.set_value((float)(plan.d[pos].minutes));
          call.perform();
          minutes_number->publish_state((float)(plan.d[pos].minutes));
       }
       {
          auto call = termo_number->make_call(); // грузим данные в контрол температуры
          call.set_value(plan.d[pos].temp());
          call.perform();
          termo_number->publish_state(plan.d[pos].temp());
       }
       plan_staff=false; // активируем обработку в контролах
    }

/////////////////////// ОТПРАВКА ИСХОДЯЩИХ ПАКЕТОВ ///////////////////////

// компоновка и отправка пакета термостату
    void sendCommand(uint8_t comm, uint8_t length=0, uint8_t* data=nullptr){
       if (_tuya_serial!=nullptr){
          uint8_t controll_buffer[length+1+6]={COMMAND_START[0],COMMAND_START[1],0,0,0,0};           
          uint8_t chkSum=COMMAND_START[0]+COMMAND_START[1]; //контрольная сумма
          controll_buffer[3]=comm; // команда
          controll_buffer[5]=length; //размер буфера данных
          _tuya_serial->write_array(controll_buffer,6); // отправляем хидер
          chkSum+=comm;
          chkSum+=length;
          _tuya_serial->write_array(data,length); // отправляем тело
          for(uint8_t i=0; i<length; i++){ // считаем КС
             chkSum+=data[i];
          }
          memcpy(controll_buffer+6, data, length); // сохраняем данные для лога
          _tuya_serial->write_array(&chkSum,1); // отправляем КС
          controll_buffer[length+6]=chkSum;
          _debugPrintPacket(controll_buffer, length+6+1);          
          lastSend=esphome::millis(); //время отправки последнего байта
          sendRight=false; // обязательно дождаться таймаута
          waitReply=comm; // запоминаем команду которую отправили
       } else {
          uint32_t timer=-10000;
          if(esphome::millis()-timer>10000){
             timer=esphome::millis();
             ESP_LOGE(TAG,"There is no UART port, work is impossible");
          }
       }
    }

//отправка статусных запросов
    void inline sendComm(tCommand comm){
       ESP_LOGV(TAG,"Send short command: %u", comm);
       sendCommand((uint8_t)comm);  
    }

//отправка  статусных команд
    void sendComm(tMode type, tStat state){
       ESP_LOGV(TAG,"Send status command: 0x%04X - %u", type, state);
       uint8_t setComm[]={(uint8_t)(((uint16_t)type)/0x100), (uint8_t)type, 0x00, 0x01, (uint8_t)state};  
       sendCommand(0x06, sizeof(setComm), setComm);   
    }

//отправка инфо о статусе модуля связи, это влияет на отдачу данных в модуль связи
    void sendMainState(tMainState state) {
       ESP_LOGV(TAG,"Send main status: %u", state);
       uint8_t temp=(uint8_t)state;
       sendCommand(0x03, 1, &temp);
    }

// отправка непонятной хрени
    void setBlabla(){
       ESP_LOGV(TAG,"Send Bla-bla-bla reply");
       uint8_t BlaBla=0x04;
       sendCommand(0x2B,1,&BlaBla);   
    }

// отправка времени термостату
    void setDeviceTime() {
       if(time_!=nullptr){
          uint8_t dateTime[]={0x01, 0, 0, 0, 0, 0, 0, 0};
          auto now = time_->now(); // в устройство льем время в соответствии с временной зоной
          if (!now.is_valid()) {
             ESP_LOGE(TAG,"System time not syncing !");
         } else {
             now.recalc_timestamp_utc(true);  // calculate timestamp of local time
             const uint8_t day_core[]={0,7,1,2,3,4,5,6}; // счет 1..7, у нас воскр-7, у них воскр-1
             dateTime[1]=now.year%100;
             dateTime[2]=now.month;
             dateTime[3]=now.day_of_month;
             dateTime[4]=now.hour;
             dateTime[5]=now.minute;
             dateTime[6]=now.second;
             dateTime[7]=day_core[now.day_of_week];
             ESP_LOGV(TAG,"Send time");
             sendCommand(0x1C, sizeof(dateTime), dateTime);
         }
       }   
    }

// отправка целевой температуры
    void setTargetTemp(uint8_t temp){
       ESP_LOGV(TAG,"Send target temperature: %f",getTemp(temp));
       uint8_t setTemp[]={0x02, 0x02, 0x00, 0x04, 0x00, 0x00, 0x00, temp};
       sendCommand(0x06,sizeof(setTemp), setTemp);   
    }

// отправка данных о расписании
    void setShedule(sPeriods* shedule){
       ESP_LOGV(TAG,"Send AUTO plan");
       uint8_t setShed[58]={0x65, 0x00, 0x00, 0x36};
       memcpy(setShed+4,shedule,sizeof(sPeriods));
       sendCommand(0x06,sizeof(setShed), setShed);
    }

/////////////////////// РАЗБОР ДАННЫХ ОТ MCU ТЕРМОСТАТА //////////////////////////

// Статусные команды
//                       receivedCommand[6], receivedCommand[5]

    bool processStatusCommand(uint8_t cByte, uint8_t commandLength) {
       bool get_change=false;
       if (cByte == 0x66 && commandLength==8){ // температура выносного датчика 
       // НОВЫЕ ВЕРСИИ ТЕРМОСТАТА МОГУТ НЕ ВЫДАВАТЬ ТЕМПЕРАТУРУ, ПАКЕТ ЕСТЬ, А В ПАКЕТЕ ЗНАЧЕНИЕ = 0, А ЭО ЗНАЧИТ, ЧТО ЕГО НЕТ
          ESP_LOGV(TAG,"Get EXTERNAL temperature %f",getTemp(receivedCommand[13]));
          if(curr_ext_temp_raw!=receivedCommand[13] && receivedCommand[13]!=0){ // температура изменилась 
             curr_ext_temp_raw=receivedCommand[13];
             get_change=true;
             float temp=getTemp(curr_ext_temp_raw);
             ESP_LOGV(TAG,"New EXTERNAL temperature %f",temp);
             if(sensor_external_temperature_!=nullptr){
                sensor_external_temperature_->publish_state(temp);
             }
          }        
       } else if (cByte == 0x1 && commandLength==5){ //ON-OFF
          ESP_LOGV(TAG,"GET ON/OFF state %u",receivedCommand[10]);
          if(this->_optimistic && send_on!=UNDEF){ // в очереди установлена комана изменения режима работы
             receivedCommand[10]=send_on;
          }
          if(_on!=receivedCommand[10]){ // состояние питания изменилось
             _on=receivedCommand[10];
             get_change=true;
             ESP_LOGV(TAG,"Change ON to %u", _on);
          }        
       } else if (cByte == 0x2 && commandLength==8){ // целевая температура
          ESP_LOGV(TAG,"GET TARGET temperature %f", getTemp(receivedCommand[13]));
          if(this->_optimistic && new_target_temp_raw!=0xFF){ // в очереди установлена тнмпература для отправки
             receivedCommand[13]=new_target_temp_raw; // подменяем температуру в пакете
          }
          if(target_temp_raw!=receivedCommand[13] && receivedCommand[13]!=0){ // показания изменились
             target_temp_raw=receivedCommand[13];
             get_change=true;
             float temp=getTemp(target_temp_raw);
             ESP_LOGV(TAG,"New TARGET temperature %f", temp);
          }        
       } else if (cByte == 0x3 && commandLength==8){ // температура внутеннего датчика
          ESP_LOGV(TAG,"GET INTERNAL temperature %f", getTemp(receivedCommand[13]));
          if(curr_int_temp_raw!=receivedCommand[13] && receivedCommand[13]!=0){ // показания изменились
             curr_int_temp_raw=receivedCommand[13];
             get_change=true;
             float temp=getTemp(curr_int_temp_raw);
             ESP_LOGV(TAG,"New INTERNAL temperature %f",temp);
             if(sensor_internal_temperature_!=nullptr){
                sensor_internal_temperature_->publish_state(temp);
             }
             this->current_temperature = temp; // публикация температуры в виджете кондея
             this->publish_state();
          }        
       } else if (cByte == 0x4 && commandLength==5){ // режим мануал или по расписанию
          ESP_LOGV(TAG,"GET MANUAL/AUTO state %u", receivedCommand[10]);
          if(this->_optimistic && send_manual!=UNDEF){ // в очереди команда на изменение
             receivedCommand[10]=send_manual; // подменяем данные
          }
          if(manualMode!=receivedCommand[10]){ // показания изменились
             manualMode=receivedCommand[10];
             get_change=true;
             ESP_LOGV(TAG,"Change MANUAL to %u", manualMode);
          }        
       } else if (cByte == 0x5 && commandLength==5){ // режим ECO
          ESP_LOGV(TAG,"GET ECO state %u", receivedCommand[10]);
          if(this->_optimistic && send_eco!=UNDEF){ // в очереди установлена комана изменения режима работы
             receivedCommand[10]=send_eco;
          }
          if(ecoMode!=receivedCommand[10]){ // показания изменились
             ecoMode=receivedCommand[10];
             get_change=true;
             ESP_LOGV(TAG,"Change ECO to %u", ecoMode);
          }        
       } else if (cByte == 0x6 && commandLength==5){ // режим LOCK
          ESP_LOGV(TAG,"Get LOCK state: %u", receivedCommand[10]);
          if(this->_optimistic && send_lock!=UNDEF){ // в очереди установлена комана изменения режима работы
             receivedCommand[10]=send_lock;
          }
          bool new_state=(receivedCommand[10]==1);// полученый статус
          if(lock_switch!=nullptr && lock_switch->state!=new_state){
             lock_switch->publish_state(new_state);    
          }
          get_change=true;
       } else if (cByte == 0x65 && commandLength==58){ // расписание
          ESP_LOGV(TAG,"GET PERIOD settings");
          if(memcmp((uint8_t*)(&plan),&(receivedCommand[10]),sizeof(plan))!=0){
             memcpy((uint8_t*)(&plan),&(receivedCommand[10]),sizeof(plan));
             ESP_LOGV(TAG,"Get new periods settings");
             refresh_controls(current_select_pos);
             get_change=true;
          }
       } else if (cByte == 0x67 && commandLength==5){ // неизвестный переключатель
          ESP_LOGV(TAG,"Get UNKNOWN_67 state: %u", receivedCommand[10]);
          if(unknowMode_67!=receivedCommand[10]){ // показания изменились
             unknowMode_67=receivedCommand[10];
             ESP_LOGE(TAG,"Get new UNKNOWN_67 value: %u",unknowMode_67);
          } 
       } else if (cByte == 0x68 && commandLength==5){ // неизвестный переключатель
          ESP_LOGV(TAG,"Get UNKNOWN_68 state: %u", receivedCommand[10]);
          if(unknowMode_68!=receivedCommand[10]){ // показания изменились
             unknowMode_68=receivedCommand[10];
             ESP_LOGE(TAG,"Get new UNKNOWN_68 value: %u",unknowMode_68);
          } 
       } else { // такую команду не знаем
          ESP_LOGE(TAG,"Command  0x%x - unknown", cByte);
          return false;
       } 
       if(get_change){
           stateChanged();
       }
       return true;
    }


// Разбор служебных пакетов
//                       receivedCommand[3], receivedCommand[5]
    bool processCommand(uint8_t commandByte, uint8_t length) {
       bool knownCommand = false;
       switch (commandByte) {
         case 0x00: {
           switch (receivedCommand[6]) {
             case 0x00 : //55 aa 01 00 00 01 00: first heartbeat
                ESP_LOGV(TAG,"First heartbeat");
                knownCommand = true;
                break;
             case 0x01 : //55 aa 01 00 00 01 01: every heartbeat after
                ESP_LOGV(TAG,"Every heartbeat after %d",esphome::millis());
                knownCommand = true;
                break;
           }
           break;
         }
         case 0x01: { // идентификатор изделия
           String id=getStringFromBuff((uint8_t*)receivedCommand+6,length);
           ESP_LOGV(TAG,"Product info received: %s", id.c_str());
           if(sensor_mcu_id_!=nullptr){
              sensor_mcu_id_->publish_state(id.c_str());   
           }
           knownCommand = true;
           break;
         }
         case 0x02: { //0x55 0xAA  0x01  0x02  0x00  0x00  0x02 , ПОХОЖЕ НА РЕСЕТ ПРОВЕРИТЬ ДЛИННОЕ НАЖАТИЕ
           ESP_LOGV(TAG,"Get reset signal");
           if (receivedCommand[5] == 0x00) { // у большинства термостатов это означает , что необходимо запростить данные
             sendCounter=4; // переход к передаче статуса wifi
             knownCommand = true;
           }
           break;
         }
         case 0x03: { //55 aa 01 03 00 00,  ОТВЕТ ТЕРМОСТАТА НА ИНФОРМИРОВАНИЕ О СТАТУСЕ WIFI 
           ESP_LOGV(TAG,"Reply on inform WIFI state");
           knownCommand = true;
           break;
         }
         case 0x04: { //55 aa 01 04 00 00  ВОЗМОЖНО СИГНАЛ О НЕОБХОДИМОСТИ СБРОСА НАСТРОЙКИ
           ESP_LOGV(TAG,"Setting reset");
           sendComm(REPRES); // отвечаем что приняли ресет
           //sendCounter=1; // запускаем карусель
           knownCommand = true;
           break;
         }
         case 0x05: {  // ресет - мигает экран
           //55 aa 01 05 00 00
           ESP_LOGV(TAG,"Reset WiFi selection");
           knownCommand = true;
           break;
         }
         case 0x1C: {  //55 aa 01 1c 00 00 1c запрос времени
           ESP_LOGV(TAG,"Date/time request");
           knownCommand = true;
           setDeviceTime(); // отвечаем временем
           break;
         }
         case 0x2B: {  //55;AA;03;2B;00;00;2D
           ESP_LOGV(TAG,"Network status request");
           knownCommand = true;
           setBlabla();
           break;
         }
       }
       return knownCommand;
     }

// РАЗБОР КОМАНДЫ ПОСЛЕ ПРОВЕРКИ СТРУКТУРЫ
    void processSerialCommand() {
       _debugPrintPacket(receivedCommand, receiveIndex+1, false);
       if (commandLength > -1) {
          bool knownCommand = false;
          if (receivedCommand[3] == 0x07) { // пакет о статусе
            knownCommand = processStatusCommand(receivedCommand[6], receivedCommand[5]);
          } else { // служебный пакет
            knownCommand = processCommand(receivedCommand[3], receivedCommand[5]);
          }
          if (!knownCommand) {
            ESP_LOGE(TAG,"Unknown command");
            _debugPrintPacket(receivedCommand, receiveIndex+1, false);
          }
       }
    }

    // сюда пихаем байт присланый MCU термостата
    void inData(uint8_t inChar, uint32_t now){
      if((int16_t)(sizeof(receivedCommand))>receiveIndex){ // контроль переполнения буфера входных данных
         receiveIndex++;
      }
      lastRead=esphome::millis();//время чтения последнего байта
      receivedCommand[receiveIndex] = inChar;
      if(receiveIndex==0 && COMMAND_START[0] != inChar){ //проверка хидера 55
         resetAll(); //  не совпало - ресетимся
      } else if (receiveIndex==1) { // проверка хидера 55 AA
        if (COMMAND_START[1]!=inChar) {
           if(COMMAND_START[0]==inChar){ // если опять 55
              receiveIndex=0; // считаем, что это нулевой байт пакета
          } else {
              resetAll(); //  не совпало - ресетимся
           }
        }
      } else if (receiveIndex == 5) { // считаем размер пакета
        commandLength = receivedCommand[4] * 0x100 + inChar;
      } else if ((commandLength > -1) && (receiveIndex == (6 + commandLength))) { // получили полный пакет
        uint8_t expChecksum = 0;         //проверяем КС
        for (uint8_t i = 0; i < receiveIndex; i++) {
          expChecksum += receivedCommand[i];
        }
        if (expChecksum == receivedCommand[receiveIndex]) {
          processSerialCommand();
          if(waitReply==receivedCommand[3]){ // пришел ответ на запрос
             sendRight=true; // можно отправлять новые пакеты     
          }
        }
        resetAll();
      }
    }

    // вызывается для публикации нового состояния климата
    void stateChanged() {
        bool need_publish=false; // флаг необходимости публикации
        // режим работы
        auto old_temp_mode=this->mode;
        if(_on==(uint8_t)OFF){ //выключено
           this->mode = climate::CLIMATE_MODE_OFF;
        } else if(_on==(uint8_t)ON) { //включен
           if(manualMode==(uint8_t)ON) { 
              this->mode = climate::CLIMATE_MODE_HEAT;
           } else if(manualMode==(uint8_t)OFF) {
              this->mode = climate::CLIMATE_MODE_AUTO;    
           }
        }
        if(old_temp_mode!=this->mode){
           need_publish=true;
        }
        //целевая температурв
        uint8_t temp_dest;
        auto now = time_->now(); // текущее время
        if((manualMode==(uint8_t)OFF || this->mode==climate::CLIMATE_MODE_AUTO) && now.is_valid()){ // режим работы по расписанию и есть правильное текущее время
           now.recalc_timestamp_utc(true);  // получить локальное время
           const uint8_t day_core[]={0,7,1,2,3,4,5,6}; // счет 1..7, у нас воскр-7, у них воскр-1
           temp_dest=plan.get_plan_current_temp_raw(day_core[now.day_of_week], now.hour, now.minute);
        } else { // ручной режим
           temp_dest=target_temp_raw;   
        }
        float temp=getTemp(temp_dest);
        if(ecoMode==(uint8_t)ON){ // если задействован режим ЭКО
            if(temp>temperature_eco){
               temp=temperature_eco;
            }                
        }
        // публикация температуры при необходимости
        if(this->target_temperature != temp){
           this->target_temperature = temp;
           need_publish=true;
        }
        
        // публикация текущей температуры при необходимости
        temp=getTemp(curr_int_temp_raw);
        if(this->current_temperature != temp){
           this->current_temperature = temp;
           need_publish=true;
        }
        // пресет
        auto old_temp_preset=this->preset;
        if(ecoMode==(uint8_t)ON) { 
           this->preset = climate::CLIMATE_PRESET_ECO;
        } else if(ecoMode==(uint8_t)OFF) {
           this->preset = climate::CLIMATE_PRESET_NONE;    
        }
        if(old_temp_preset!=this->preset){
           need_publish=true;   
        }
        // Текущий режим работы (показометр)
        auto old_temp_action=this->action;
        
        if(curr_ext_temp_raw!=0xFF){ // если есть показания внешнего датчиика, то работаем по ним
           temp=getTemp(curr_ext_temp_raw); // температура внешнего датчика
        } else { // если показаний внешнего датчика нет, работаем по внутреннему
           temp=getTemp(curr_int_temp_raw); // температура внутреннего датчика
        }
        
       
        if(this->mode == climate::CLIMATE_MODE_OFF){
           this->action= climate::CLIMATE_ACTION_OFF;  
        } else {
           if(old_temp_action==climate::CLIMATE_ACTION_HEATING){ // было в нагреве
              if(this->current_temperature >= this->target_temperature ||
                 temp >= temperature_overheat){ // температура превысила целевую или перегрелся внешний датчик
                 this->action = climate::CLIMATE_ACTION_IDLE; // значит прекратить нагрев
              }
           } else if(old_temp_action==climate::CLIMATE_ACTION_IDLE ){
              if(this->current_temperature <= this->target_temperature-temperature_deadzone &&
                 temp < temperature_overheat ){ // температура ниже целевой на гистерезис
                 this->action = climate::CLIMATE_ACTION_HEATING; // нагрев включается
              }
           } else if(old_temp_action==climate::CLIMATE_ACTION_OFF){
              if(this->current_temperature <= this->target_temperature-temperature_deadzone &&
                 temp < temperature_overheat){ // температура ниже целевой на гистерезис
                 this->action = climate::CLIMATE_ACTION_HEATING; // нагрев включается
              } else {
                 this->action = climate::CLIMATE_ACTION_IDLE;
              }
           }               
        }
        if(old_temp_action!=this->action || need_publish){ // бликуем при необходимости
           ESP_LOGV(TAG,"State changed, %f, %f",this->current_temperature, temp);
           this->publish_state();
        }
    }

    // вызывается пользователем из интерфейса ESPHome или Home Assistant
    void control(const esphome::climate::ClimateCall &call) override {
        ESP_LOGV(TAG,"State changed from user.");
        bool need_pub=false;
        // Проверка режима
        auto _mode=this->mode;
        if (call.get_mode().has_value()) {
            ClimateMode mode = *call.get_mode();
            switch (mode) {
                case climate::CLIMATE_MODE_OFF:
                    if(_on!=OFF) send_on=OFF;
                    this->mode = mode;
                    break;
                case climate::CLIMATE_MODE_HEAT:
                    if(_on!=ON) send_on=ON;
                    if(manualMode!=ON) send_manual=ON;
                    this->mode = mode;
                    break;
                case climate::CLIMATE_MODE_AUTO:
                    if(_on!=ON) send_on=ON;
                    if(manualMode!=OFF) send_manual=OFF;
                    this->mode = mode;
                    break;
                default:
                    break;
            }
        }
        // Проверка пресета
        auto _presset=this->preset;
        if (call.get_preset().has_value()) {
            ClimatePreset preset = *call.get_preset();
            switch (preset) {
                case climate::CLIMATE_PRESET_ECO:
                    if(ecoMode!=ON) send_eco=ON;
                    this->preset = preset;
                    break;
                case climate::CLIMATE_PRESET_NONE:
                    if(ecoMode!=OFF) send_eco=OFF;
                    this->preset = preset;
                    break;
                default:
                    break;
            }
        } 
        // Проверка целевой температуры
        auto t_temp=this->target_temperature;
        if (call.get_target_temperature().has_value()) {
            this->target_temperature=*call.get_target_temperature();
            if((_on==ON && manualMode==ON) || this->mode == climate::CLIMATE_MODE_HEAT) { // целевую температуру меняем только в режиме нагрева
               new_target_temp_raw= (uint8_t)(this->target_temperature*2); // новая целевая температура, для отправки термостату
            } else {
               this->publish_state();
            }
        }
        if(this->_optimistic || t_temp!=this->target_temperature || _presset!=this->preset || _mode!=this->mode){
           stateChanged();
        }
    }

   public:
    // инициализация объекта
    void initTermo(esphome::uart::UARTComponent *parent = nullptr) {
        _tuya_serial = parent;
        // первоначальная инициализация
        this->preset = climate::CLIMATE_PRESET_NONE;
        this->mode = climate::CLIMATE_MODE_OFF;
        this->action = climate::CLIMATE_ACTION_IDLE;
        // заполнение шаблона параметров отображения виджета
        _traits.set_supports_current_temperature(true);
        _traits.set_supported_modes(this->_supported_modes);
        _traits.set_supported_presets(this->_supported_presets);
        _traits.add_supported_mode(ClimateMode::CLIMATE_MODE_OFF);
        _traits.add_supported_preset(ClimatePreset::CLIMATE_PRESET_NONE);
        _traits.set_supports_action(true);
    };

    float get_setup_priority() const override { return esphome::setup_priority::DATA; }
    void set_product_id_text(text_sensor::TextSensor *sensor){sensor_mcu_id_=sensor;}
    void set_external_temperature_sensor(sensor::Sensor *sensor) { sensor_external_temperature_ = sensor; }
    void set_internal_temperature_sensor(sensor::Sensor *sensor) { sensor_internal_temperature_ = sensor; }
    void set_visual_min_temperature_override(float val){_traits.set_visual_min_temperature(val);}
    void set_visual_max_temperature_override(float val){_traits.set_visual_max_temperature(val);}
    void set_visual_temperature_step_override(float val){_traits.set_visual_temperature_step(val);}
    void set_visual_temperature_eco(float val){this->temperature_eco=val;}
    void set_visual_temperature_overheat(float val){this->temperature_overheat=val;}
    void set_visual_temperature_deadzone(float val){this->temperature_deadzone=val;}
    void set_optimistic(bool optimistic) { this->_optimistic = optimistic;}
    bool get_optimistic() { return this->_optimistic; }

    // подключение чилдрен лок
    void set_children_lock_switch(TuyaTermo_Switch* switch_){ // подключение свитча children lock
       lock_switch=switch_;
       switch_->add_on_state_callback([this](bool st){ 
            // обработка переключения свитча
            if(!std::isnan(st)){
               if(lock_switch->state!=st){ // новое состояние свича
                   if(st){
                       send_lock=ON;
                   } else {
                       send_lock=OFF;
                   }
               }
            }
       });
    }

    // контролл установки часов периода
    void set_hours_number(TuyaTermo_Number *number_){
       hours_number=number_;
       number_->add_on_state_callback([this](float new_value){ 
          if(plan_staff==false){ //только если данные изменились не в момент переключения селекта
             plan.d[current_select_pos].hours=(uint8_t)new_value;
             timer_plan_change=esphome::millis(); // таймер изменения данных
          }
       });
    }

    // контролл установки минут периода
    void set_minutes_number(TuyaTermo_Number *number_){
       minutes_number=number_;
       number_->add_on_state_callback([this](float new_value){ 
          if(plan_staff==false){ //только если данные изменились не в момент переключения селекта
             plan.d[current_select_pos].minutes=(uint8_t)new_value;
             timer_plan_change=esphome::millis(); // таймер изменения данных
          }
       });
    }

    // контролл установки температуры периода
    void set_temperatures_number(TuyaTermo_Number *number_){
       termo_number=number_;
       termo_number->traits.set_min_value(_traits.get_visual_min_temperature());
       termo_number->traits.set_max_value(_traits.get_visual_max_temperature());
       //termo_number->traits.set_step(_traits.get_visual_temperature_step());
       termo_number->traits.set_step(_traits.get_visual_target_temperature_step());
       number_->add_on_state_callback([this](float new_value){ 
          if(plan_staff==false){ //только если данные изменились не в момент переключения селекта
             plan.d[current_select_pos].temp(new_value);
             timer_plan_change=esphome::millis(); // таймер изменения данных
          }
       });
    }

    // селект выбора периода    
    void set_plan_select(TuyaTermo_Select *select_){
       plan_select=select_;
       select_->traits.set_options(str_plan);
       select_->add_on_state_callback([this](std::string new_value,  unsigned int pos){ 
          if(current_select_pos != pos){
             refresh_controls(pos);
             current_select_pos=pos; 
          }
       });
       //select_->publish_state(w1); //дергаем первый раз для инициализации
       auto call = plan_select->make_call(); // грузим данные в контрол часов
       call.set_option(w1);
       call.perform();
       timer_plan_change=0;
       refresh_controls(1);
    }

    // детский замок  "НУ НЕ СМОГЛА Я" (цы)
    //void set_children_lock(TuyaTermo_Lock *lock_){
    //   child_lock=lock_;
    //   lock_->add_on_state_callback([this](void){ // видимо просто дергается этот кусок кода
    //      // тут проверить состояние и обработать
    //   });
    //}
 
    // часы
    void set_time(time::RealTimeClock *time) { this->time_ = time; };
    
    // нога входного сигнала ресет протокола
    void set_input_reset_pin(GPIOPin  *pin = nullptr){ 
       pin->setup(); // захватываем ногу
       pin->pin_mode(gpio::FLAG_INPUT); // настраиваем ее на выход
       in_reset_pin_state=pin->digital_read(); // читаем состояние
       in_reset_pin=pin; // запоминаем пин
    }  

    // вывод в дебаг текущей конфигурации компонента
    void dump_config() {
        ESP_LOGCONFIG(TAG, "Tuya Termostate:");
        LOG_TEXT_SENSOR("  ", "MCU product ID", this->sensor_mcu_id_);
        ESP_LOGCONFIG(TAG, "  Firmware version: %s", TERMO_FIRMWARE_VERSION.c_str());
        ESP_LOGCONFIG(TAG, "Optimistic: %s", TRUEFALSE(this->_optimistic));
        LOG_SENSOR("  ", "Internal Temperature", this->sensor_internal_temperature_);
        LOG_SENSOR("  ", "External Temperature", this->sensor_external_temperature_);
        LOG_SWITCH("  ", "Children lock switch", this->lock_switch);
        if(plan_select!=nullptr){
           ESP_LOGCONFIG(TAG, "  For Shedule in AUTO mode:");
           LOG_SELECT("  ","  Period Selector",this->plan_select);
           LOG_NUMBER("  ","   Start Hours", this->hours_number);
           LOG_NUMBER("  ","   Start Minutes", this->minutes_number);
           LOG_NUMBER("  ","   Temperature", this->termo_number);
        }      
        this->dump_traits_(TAG);
        ESP_LOGCONFIG(TAG, "  [x] Additional setting:");
        ESP_LOGCONFIG(TAG, "      - Eco: %.2f", temperature_eco);
        ESP_LOGCONFIG(TAG, "      - Overheat: %.2f", temperature_overheat);
        ESP_LOGCONFIG(TAG, "      - Deadzone: %.2f", temperature_deadzone);
    }

    // как оказалось сюда обращаются каждый раз для получения любого параметра
    // по этому имеет смысл держать готовый объект
    esphome::climate::ClimateTraits traits() override {
        return _traits;
    }

    // возможно функции get и не нужны, но вроде как должны быть
    void set_supported_modes(const std::set<ClimateMode> &modes) { this->_supported_modes = modes; }
    std::set<ClimateMode> get_supported_modes() { return this->_supported_modes; }

    void set_supported_presets(const std::set<ClimatePreset> &presets) { this->_supported_presets = presets; }
    const std::set<climate::ClimatePreset> &get_supported_presets() { return this->_supported_presets; }

    void setup() override{
        sendCounter=1;
    }  

    void loop() override {
        
        uint32_t now=esphome::millis();
        
        // проверяем входящий сигнал ресета
        if(in_reset_pin!=nullptr){
           uint8_t val=in_reset_pin->digital_read();
           if(in_reset_pin_state!=val){
             in_reset_pin_state=val;
             reset_timer=now;
           } else if(now-reset_timer>2000){ // изменение сигнала было больше 2 сек назад
              ESP_LOGV(TAG,"Get inbound RESET, restart protocol interchange");
              reset_timer=0;
              sendCounter=1; // начать крутить шарманку опроса с начала
           }
        }

        static uint32_t cycle_reset_timer=0;
        if (now-cycle_reset_timer>PROTO_RESTART_UNTERVAL*60*1000){ // каждые 20 минут опрашиваем MCU c нуля
           ESP_LOGV(TAG,"Cuclic auto restart protocol interchange");
           cycle_reset_timer=now;
           sendCounter=1; // начать крутить шарманку опроса с начала
        }

        // читаем UART порт
        if (_tuya_serial!=nullptr){
           if(_tuya_serial->available()){ //читаем
              uint8_t data;
              _tuya_serial->read_byte(&data);
              inData(data, now); // обрабатываем
           }               
        }

        // при работе в режиме по расписанию для отслеживания индикации режима
        if(manualMode==OFF){
           static uint32_t timer=0;
           if(now-timer>60000){ // проверяем раз в минуту, расписание то с минимальным шагом в минуту
              stateChanged();
              timer=now;              
           }
        }
        
        // карусель обмена данными
        if(now-lastSend>UART_TIMEOUT && (now-lastRead>UART_TIMEOUT || sendRight)){ //можно отправлять
          sendRight=false;
          if(sendCounter==1){ // отправим первый heatbear
             sendComm(PING); 
             sendCounter++;
          } else if(sendCounter==2){
             sendComm(IDENT);
             sendCounter++;
          } else if(sendCounter==3){
             sendComm(STAT);
             sendCounter++;
          } else if(sendCounter==4){
             sendComm(DATA);
             sendCounter++;
          } else if(sendCounter==5){
             sendMainState(wsPROC);  // непоянтно зачем это и как должно работать, смысла не вижу значек wifi проявить не удалось
             sendCounter++;
          } else if(sendCounter==6){ 
             sendMainState(wsOK); // непоянтно зачем это и как должно работать, смысла не вижу значек wifi проявить не удалось
             sendCounter++;
          } else if(sendCounter==7){ 
             sendMainState(wsOK_4); //этот статус вызывает отдачу первого блока данных
             sendCounter++;
          } else if(sendCounter==8){ 
             sendMainState(wsOK_5); // этот статус включает отдачу второго блока данных и периодическую показаний термодатчиков
             sendCounter++;
          } else if(sendCounter==9){ // остальной процессинг
             if(new_target_temp_raw!=0xFF){  // нужно установить новую целевую температуру
                setTargetTemp(new_target_temp_raw);
                new_target_temp_raw=0xFF;
             } else if(send_lock!=UNDEF){ // состояние LOCK изменилось
                sendComm(LOCK, send_lock);
                send_lock=UNDEF;
             } else if(send_eco!=UNDEF){ // состояние режима ECO изменилось
                sendComm(ECO, send_eco);
                send_eco=UNDEF;
             } else if(send_manual!=UNDEF){ // состояние режима AUTO изменилось
                sendComm(POWER, ON);
                sendComm(MANUAL, send_manual);
                send_manual=UNDEF;
             } else if(send_on!=UNDEF){ // состояние режима ON/OFF изменилось
                sendComm(POWER, send_on);
                send_on=UNDEF;
             } else if(timer_plan_change && now-timer_plan_change>SET_PLAN_TIMEOUT*1000){ // изменилось расписание
                timer_plan_change=0;
                setShedule(&plan);
                stateChanged();
             } else if(now-lastSend>HEARTBEAT_INTERVAL*1000){
                sendComm(PING);  
             }
          }
        }
        
    };
 
    friend class TuyaTermo_Switch;
    friend class TuyaTermo_Number;
    friend class TuyaTermo_Select;
    friend class TuyaTermo_Lock;
};

}  // namespace 
}  // namespace esphome

#endif //TUYA_TERMO_H
