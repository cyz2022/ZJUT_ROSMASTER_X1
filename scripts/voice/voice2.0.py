import pygame

def play_music(choice):
    try:
        pygame.mixer.init()
        pygame.mixer.music.set_volume(1.0)

        if choice.isdigit():
            number_or_file = int(choice)
            if number_or_file < 2 or number_or_file > 8:
                print("输入的数字必须在2到8之间。")
                return
            file_path = f"{number_or_file}.mp3"
        elif isinstance(choice, str):
            if choice == 's':
                file_path = 'scenic.mp3'
            elif choice == 't':
                file_path = 'treasure.mp3'
            elif choice == 'p':
                file_path = 'prepared.mp3'
            else:
                print("无效的指令或文件名。")
                return
        else:
            print("无效的参数类型。")
            return

        pygame.mixer.music.load(file_path)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
    except KeyboardInterrupt:
        pygame.mixer.music.stop()


if __name__ == "__main__":
    while True:
        try:
            choice = input("请输入指令：")
            play_music(choice)
        except ValueError:
            print("请输入有效的指令。")
