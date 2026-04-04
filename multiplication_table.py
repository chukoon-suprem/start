def print_multiplication_table(n):
    print(f"\n=== {n}단 ===")
    for i in range(1, 10):
        print(f"{n} x {i} = {n * i}")

def main():
    print("구구단 프로그램")
    print("=" * 20)
    print("1. 특정 단 출력")
    print("2. 전체 구구단 출력 (2~9단)")
    print("=" * 20)

    choice = input("선택하세요 (1 또는 2): ").strip()

    if choice == "1":
        try:
            n = int(input("몇 단을 출력할까요? (2~9): "))
            if 2 <= n <= 9:
                print_multiplication_table(n)
            else:
                print("2~9 사이의 숫자를 입력하세요.")
        except ValueError:
            print("올바른 숫자를 입력하세요.")
    elif choice == "2":
        for n in range(2, 10):
            print_multiplication_table(n)
    else:
        print("올바른 선택이 아닙니다.")

if __name__ == "__main__":
    main()
