import xlsxwriter



def main():
  
  workbook = xlsxwriter.Workbook('Expenses01.xlsx')
  worksheet = workbook.add_worksheet()
  
if __name__ == "__main__":
  main()
