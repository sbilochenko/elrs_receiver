1) переконайтесь що ваш приймач (RX) робочий
2) для подальшої роботи RX має біндитися з апаратурою керування - перевірте bind перед тим як рухатися далі
   якщо нема бінда - нема що перевіряти далі
3) <img width="816" height="760" alt="image" src="https://github.com/user-attachments/assets/3be57da5-d14a-4688-8c30-e75b6f0a9a06" />
знайдіть розпіновку вашого ESP32, 
перевірено роботу при підключенні до GPIO16 - cюди приєднуємо TX приймача

4) [Код тесту роботи ESP+ELRS RX](test/main_test_elrs.cpp)
5) Коли апаратура та RX зв'язані і між ними є bind - рухайте стіками апаратури керування і у serial monitor має бути видно дані, <img width="798" height="323" alt="{E5F33828-43BD-438E-B7E9-83EA40AECC58}" src="https://github.com/user-attachments/assets/513a9323-3bb5-4efd-9e8d-ba7fe69ddd16" />
6) далі можна користуватися [файлом main.cpp](src/main,cpp)  цього репозиторію
