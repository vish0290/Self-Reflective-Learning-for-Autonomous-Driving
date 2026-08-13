import math

def isPrime(n):
    if n < 2:
        return False
    for i in range(2, int(math.sqrt(n)) + 1):
        if n % i == 0:
            return False
    return True


import requests
from bs4 import BeautifulSoup

url = "https://docs.google.com/document/d/e/2PACX-1vSvM5gDlNvt7npYHhp_XfsJvuntUhq184By5xO_pA4b_gCWeXb6dM6ZxwN8rE6S4ghUsCj2VKR21oEP/pub"

response = requests.get(url)
tree = BeautifulSoup(response.text, "html.parser")

table = tree.find("table")
rows = table.find_all("tr")

headers = [cell.get_text(strip=True) for cell in rows[0].find_all(["td", "th"])]
data = [
    dict(zip(headers, [cell.get_text(strip=True) for cell in row.find_all(["td", "th"])]))
    for row in rows[1:]
]

# Build grid
max_x = max(int(r["x-coordinate"]) for r in data)
max_y = max(int(r["y-coordinate"]) for r in data)

grid = [[" " for _ in range(max_x + 1)] for _ in range(max_y + 1)]

for r in data:
    x = int(r["x-coordinate"])
    y = int(r["y-coordinate"])
    grid[y][x] = r["Character"]

for row in grid:
    print("".join(row))
