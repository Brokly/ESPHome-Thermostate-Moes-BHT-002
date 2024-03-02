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

Из-за медленной реакции термостатов на бекенах добавлена Optimistic mode. Теперь публикация данных всегда соответствует установкам пользователя, не взирая на то применил ли установку термостат. В случае неприменимой настроки, она обновится в реальное состояние через некоторое время.

<b>UPD (02.03.24):</b>

Добавлена настройка mode_restore: true/false. По умолчанию - false. Настройка включает автовосстановление режима работы термостата после выключения питания. Восстанавливает только режим работы - Mode.
Теперь статус подключения к WIFI и серверу передаваемый термостату соответствует реальной ситуации. Ранее статус был фиктивный.





