def mystery_func(str)
  s = ""
  (str.length()/2).times do |index|
    ((str[2*index+1]).to_i).times do |dindex|
      s += str[2*index]
    end
  end
  return s
end



def numbers_sum(arr)
	v = 0
  arr.each {|element| if element.is_a? Integer
    v+=element
   end }
   return v
end

# p numbers_sum([1, 2, "13", "4", "645"])

def add_letters(a)
  v = 0
  alphabet = ("a".."z").to_a
  if a.empty?
    z = "z"
  else
    a.each do |element|
      v += alphabet.index(element)+1
    end
    r,v = v.divmod(26)
    z = alphabet[v-1]
  end
  return z
end
#p add_letters(["a", "b", "c", "d", "e", "c","a"])

def make_title(str)
  str = str.split(/ /)
	str = str.map {|element| element.capitalize()}
  return str.join(" ")
end
# p make_title("This is a title")

def duplicates(str)
	letters={}
  count = 0
  str.each_char {|chara| letters.store(chara, str.count(chara))}
  letters.each {|k,v| count += (v-1)  }
  return count
end
#p duplicates("Hello World!")

def scale_tip(arr)
  v=[0,0]
  i=0
	arr.each {|element| if element.is_a? String
    i+=1
  else
    v[i]+=element
  end
  }
  if v[0]==v[1]
    puts "balanced"
  else
  v.index(v.max) == 0 ? (puts "Left") : (puts "Right")
end
end
#scale_tip([0, 0, "I", 1, 1])

def palindrome(s)
  b = 1
  s[0] == s[(s.length-1)] ? (b*=1):(b*=0)
	s1 = s[1..(s.length-2)]
  if s1.length==1 or s1.length==0
    return !b.zero?
  elsif s1.length<=0
    puts "Invalid input"
    return false
  else
    palindrome(s1)
  end
end
 # p palindrome("abcba")
 def fact(n)
   if n<= 1
     1
   else
     n * fact( n - 1 )
   end
 end
#p fact(5)
def product_pair(arr, k)
 	if k > arr.length
    return nil
  else
    values=[0,0]
    ret=[]
    fact(arr.length+2).times do |i|
      si = []
      if (i.to_s(arr.length+1)).length==k
        si = (i.to_s(arr.length+1)).each_char.map(&:to_i)
        c = 1
        if ((si.uniq).length == si.length) and !(si.include? (arr.length))
          si.each {|element|  c*=arr[element]}
          if values[0]<= c
            values[0] = c
          elsif values[1]>= c
            values[1] = c
          end
        end
      end
    end
  end
  return values
 end

#p product_pair([1, -2, -3, 4, 6, 7], 3)

def map_letters(word)
	h = Hash[]
  (0 ... word.length).each do |i|
    if (h.keys).include?((word[i]).to_sym)
      h[(word[i]).to_sym].append(i)
    else
      h[(word[i]).to_sym] = [word.index((word[i]))]
    end
  end
  return h
end
#p map_letters("susana gonzakez marquez")
def build_staircase(height, block)
  z=Array.new(height) { Array.new(height, 0) }
	height.times do |i|
    height.times do |j|
      if i<j
        z[i][j] = "_"
      else
        z[i][j]=block
      end
    end
  end
  z.each do |element|
    p element
  end
end

def total_sales(sales_table, product)
  v = 0
	if !(sales_table[0]).include?(product)
    return "Product not found"
  else
    a = sales_table[0].index(product)
    sales_table.shift
    (sales_table).each do |element|
      v += element[a]
    end
  return v
  end
end

#total_sales([
#  ["A", "B", "C"],
#  [ 2 ,  7 ,  1 ],
#  [ 3 ,  6 ,  6 ],
#  [ 4 ,  5 ,  5 ]], "C")

def mirror_cipher(message)
	if message.length == 1
    a0 = (message.join("")).downcase
    a1 = (("a".."z").to_a).join("")
  else
    a1 = message[1].downcase
    a0 = message[0].downcase
  end
    h = Hash[]
    (a1.length).times do |i|
      if !(h.keys).include?((a1[i]).to_sym)
        h[(a1[a1.length-i-1]).to_sym] = (a1[i])
      end
    end
    (a0.length).times do |i|
       if (h.keys).include?((a0[i]).to_sym)
         a0[i]= h[(a0[i]).to_sym]
       elsif (h.values).include?((a0[i]))
         a0[i] = (h.key((a0[i]))).to_s
       end
    end
  return a0
end
#p mirror_cipher(["Mubashir Hassan", "edabitisamazing"])
def validate_subsets(subsets, set)
  v = 1
	subsets.each{|element| element.each {|elem|(set.include?(elem))?(v*=1):(v*=0)} }
  return !v.zero?
end


def possible_palindrome(str)
	h = Hash[]
  v = 1
  str.each_char do |element|
    if (h.keys).include?(element.to_sym)
      h[element.to_sym] +=1
    else
      h[element.to_sym] = 1
    end
  end
  (h.values).each do |element|
    ((h.values).count(1)>1)?(v*=0):(v*=1)
    if element%2 == 0
      v*=1
    else
      if element>=3
        v*=0
      end
    end
  end
  return !v.zero?
end
# p possible_palindrome("acabbaa")







# XD
