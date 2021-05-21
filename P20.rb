def letter_counter(arr, letter)
  count = 0
  arr.each  { |pairs| pairs.map {|element|
    if element == letter
      count+=1
  end } }
  return count
end
p letter_counter([
  ["D", "E", "Y", "H", "A", "D"],
  ["C", "B", "Z", "Y", "J", "K"],
  ["D", "B", "C", "A", "M", "N"],
  ["F", "G", "G", "R", "S", "R"],
  ["V", "X", "H", "A", "S", "S"]
], "H")
