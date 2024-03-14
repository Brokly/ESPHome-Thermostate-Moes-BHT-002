# ESPHome-Thermostate-Moes-BHT-002


Большое распространение у любителей домашней автоматизации получили WIFI термостаты Moes-BHT-002.

![1](https://user-images.githubusercontent.com/11642286/208393574-359c222d-f9fa-4e75-8af6-3d01674839ef.jpg)

Полное название  BHT-002-GBLW. Это вай-фай термостат электрического теплого пола из экосистемы Tuya.
Конечно его можно использовать в родной экосистеме. Можно подключить к другим системам с использованием различных
плгагинов и стыков. 
Я использую Home Assistant и для подключения у меня были варианты:
- Интеграция Tuya https://www.home-assistant.io/integrations/tuya/
- Интеграция Local Tuya https://github.com/rospogrigio/localtuya
- Перешить используя проект https://github.com/klausahrenberg/WThermostatBeca и подключить через MQTT
- Есть и СТАНДАРТНАЯ иинтеграция https://esphome.io/components/tuya.html

Все эти способы рабочие, но имеют недостатки. Где то обязательно нужно наличие подключения к внешним серверам,
где то вы не сможете использовать весь потенциал устройства, где то будут ошибки и непонятки. Да и ни один из
этих способов не будет 100% под вашим контролем. Поизучав последний проект я пришел к выводу, что было бы не 
плохо написать компонент для ESPHome. Тем более, что за связь в этом устройстве отвечает модуль Tuwe3S, который
оказался аналогом ESP-12. Термостат имеет процессор который обслуживает процесс нагрева и управления и по UART
обменивается данными с модулем Tuwe3S. Естественно найти полное описание протокола не удалось, в проекте
WThermostatBeca был описан лишь служебный обмен, остального нет. Пришлось расшифровывать протокол, помогло то,
что он простой и выполнен в китайском стиле :)) 

На выходе получается прошивка, которая является родной для ESPHome. Как всегда основная проблема первоначально 
залить ее в Tuwe3S. Знаю, что есть проект, который позволяет ничего не паяя заменить прошивку в устройствах от
Туя. Мне проше припаяться. Схема стандартная:

![image](https://user-images.githubusercontent.com/11642286/208415302-ad267f5a-6b39-47d1-96ba-4d81af84e8c5.png)

Конечно совсем правильная схема, примерно такая:
![image](https://user-images.githubusercontent.com/11642286/208404195-f8384adb-0c07-4520-aa52-4a8dff1be936.png)

Но "на пару раз прошить", я решил обойтись без резисторов. Вам советую все же использовать резисторы.
Передняя панель отстегнута от термостата. Питание подается внешнее 3.3 вольта. Перед заливкой прошивки нужно
кратковременно замкунть RESET на GND.

Сущности устройства:

![image](https://user-images.githubusercontent.com/11642286/208405764-30eaf520-fde5-4c25-b54b-a31cc241caed.png)

Часы синхронизируются автоматически, если задан источник времени.

Пример конфигурирования

```yaml
climate:
  - platform: tuya_termo
    name: devicename
    uart_id: id_uart_bus
    time_id: id_sync_time
    mcu_reset_pin: GPIOxx
    optimistic: true
    visual:
      min_temperature: 5
      max_temperature: 35
      eco_temperature: 20
      overheat_temperature: 45
      deadzone_temperature: 1
    internal_temperature:
      name: Internal Temperature
    external_temperature:
      name: External Temperature
    children_lock:
      name: Child Lock
    shedule:
      selector:
        name: Plan Day Selector
      hours:
        name: Plan Hours
      minutes:
        name: Plan Minutes
      temperature:
        name: Plan Temperatures
    product_id:   
      name: Product Identifier
```

<b>UPD (20.09.23):</b>

Появились версии с модулем CB3S, на базе BK7231N. В данный момент существует замечательный проект LibreTiny EspHome, надеюсь вы его легко найдете. С его помощью можно скомпилировать прошивку под новый модуль.
Код дополнен и исправлен для коректной работы с новыми версиями термостатов с чипами на BK72xx. Новые термостаты не публикуют температуру выносного датчика, поэтому у некоторых пользователей он будет в статусе "Неизвестно", просто скройте его в конигурации. 

<b>Выражаю благодарность за помощь в отладке кода Михаил Mivlz</b>.

Модули TUYA на чипе BK72xx шьются, практически, так же как и ESP8266, только тут в самом начале загрузки нужно посадить контакт CEN на землю, примерно на секунду. Конечно, не звбывайте, что вам нужна LibreTiny ESPHOME.

Подключение для прошивки:

![image](https://github.com/Brokly/ESPHome-Thermostate-Moes-BHT-002/assets/11642286/922be46b-6c74-49d4-aa16-962a422c1779)

Распайка чипа с сайта производителя:

![image](https://github.com/Brokly/ESPHome-Thermostate-Moes-BHT-002/assets/11642286/49673fa9-3ab6-495a-9b32-de093d7249b6)

Таблица назначения ног:

![image](https://github.com/Brokly/ESPHome-Thermostate-Moes-BHT-002/assets/11642286/0e64e196-e72c-41dc-b0a4-bbcccd5d0c9b)

<b>UPD (18.12.23):</b>

Из-за медленной реакции термостатов на бекенах добавлена настройка
```yaml
- platform: tuya_termo
  оptimistic: true # оr false
```
По умолчанию - true. При включеной опции публикация данных всегда соответствует установкам пользователя, не взирая на то применил ли установку термостат. В случае неприменимой настроки, она обновится в реальное состояние через некоторое время.

<b>UPD (02.03.24):</b>

Добавлена настройка 
```yaml
- platform: tuya_termo
  mode_restore: true # оr false
```
По умолчанию - false. Настройка включает автовосстановление режима работы термостата после выключения питания. Восстанавливает только режим работы - Mode. Теперь статус подключения к WIFI и серверу передаваемый термостату соответствует реальной ситуации. Ранее статус был фиктивный.

<b>UPD (07.03.24):</b>

Теперь при восстановлении режим работы после пропадания питания (mode_restore: true), восстанавливаются все оперативные параметры термостата (режим работы, пресет, целевая температура).

Добавлен выход для ресета MCU термостата. При долгом отсутствии ответа от MCU термостата, на выходе формируется инверсный импульс сброса (замыкание на GND). В нормальном состоянии выход в режиме HI-IMPEDANCE. Для использования функции перезагрузки термостата требуеются аппаратные доработки. Нужно прокинуть провод от управления модудем DC12->DC5, на выходную ножку термостата.

![image](https://github.com/Brokly/ESPHome-Thermostate-Moes-BHT-002/assets/11642286/986822d7-a11e-4a6a-9523-2422d72b2ae4)

Моя версия термостата использует преобразователь [LP6498A](http://www.lowpowersemi.com/storage/files/2023-05/7af3ba3edb6fc52b54c51add36301d7e.pdf)

Точка подключения для управления режимом работы dc/dc преобразователя
![image](https://github.com/Brokly/ESPHome-Thermostate-Moes-BHT-002/assets/11642286/62a75737-76d4-4040-a4ac-c83248175eaf)![image](https://github.com/Brokly/ESPHome-Thermostate-Moes-BHT-002/assets/11642286/44d7258b-4a69-4528-916b-e9c49c1f42ee)



```yaml
- platform: tuya_termo
  mcu_reset_pin: GPIOXX
```

Изменено конфигурирование контролов расписания
```yaml
- platform: tuya_termo
  shedule:
    selector:
      name: "Shedule Day Selector"
    hours:
      name: "Shedule Hours"
    minutes:
      name: "Sheduule Minutes"
    temperature:
      name: "Shedule Temperatures"
```

<b>UPD (12.03.24):</b>

Добавлен счетчик перезагрузок, если сконфигурирован выход управления перезагрузкой термостата, то можно добавить счетчик. Счетчик считает количество перезагрузок, при прошивке по OTA сбрасывается в ноль
```yaml
- platform: tuya_termo
  mcu_reset_pin: GPIOXX
  mcu_reload_counter:
    name: MCU Reset Counter
```

Добавлена настройка позволяющая включить/выключить пакеты синхронизации времени. Некоторым термостатам, для правильной работы часов, требуются ежесекундные пакеты, которые позволяют часам идти, в противном случае, часы стоят.
По умолчанию настрока выключена (false), ежесекундные пакеты не отправляются. Ниже пример конфигурации со включенной отправкой ежесекундных пакетов
```yaml
- platform: tuya_termo
  time_id: sync_time_id
  time_sync_packets: true
```

Добавлены настройки дополнительных выходов. Некоторые устройства требуют управление через пины. Это вход ресета протокола reset_pin, получив на этом пине изменение уровня сигнала модуль управления проводит реинициализацию, с передачей всех настроек так, как будто устройство только что включили. Так же выход status_pin, преположительно, к этому пину должен быть подключен светодиод устройства, индицирующий режим работы сети. Эти пины могут быть добавлены в конфигурацию:
```yaml
- platform: tuya_termo
  status_pin: Pxx
  reset_pin: Pxx
```
Но как правило, эти пины нужно конфигурировать по требованию MCU термостата. Если термостату требуется такая конфиигурация, то в лог будут выданы сообщения уровня ERROR и/или WARNING, в которых будут указаны нужные пины. 

Примерная полная конфигурация (не все что есть в ней нужно Вам):
```yaml
climate:
  - platform: tuya_termo
    name: ${upper_devicename}
    uart_id: uart_bus # uart шина для управления MCU термостата
    time_id: sync_time # источник синхронизации времени
    time_sync_packets: false # пакеты синхронизациии времени, для некоторых версий устройств (по умолчанию false)
    optimistic: true 
    mode_restore: true # настройка восстановления режима работы после перезагрузки (по умолчанию true)
    mcu_reset_pin: P9  # выходной пин принудительного сброса MCU термостата (только для доработанных термостатов)
      name: ${upper_devicename} MCU Reset Counter
    mcu_reload_counter: # счетчик принудительных перезагрузок MCU термостата
    reset_pin: P14 # входной пин для реинициализации протокола обмена с MCU, требуется для некоторых термостатов
    status_pin: P15 # выходной пин индикации сетевого статуса, требуется для некоторых термостатов, но можно установить светодиод
    visual: # настройки для правильного отображения виджета, в большинстве случаев используются по умолчанию
      min_temperature: 5
      max_temperature: 35 
      eco_temperature: 20
      overheat_temperature: 45
      deadzone_temperature: 1
    internal_temperature: # сенсор температуры воздуха в помещении
      name: ${upper_devicename} Internal Temperature  
    external_temperature: # внешний сенсор температуры пола
      name: ${upper_devicename} External Temperature
    children_lock:
      name: ${upper_devicename} Child Lock
    shedule: # набор контролов для управления расписанием автономной работы термостата
      selector:
        name: ${upper_devicename} Plan Day Selector
      hours:
        name: ${upper_devicename} Plan Hours
      minutes:
        name: ${upper_devicename} Plan Minutes
      temperature:
        name: ${upper_devicename} Plan Temperatures
    product_id:   
      name: ${upper_devicename} Product Identifier
```
