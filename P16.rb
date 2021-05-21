def find_bob(names)
	names.each do |element|
    if element=="Bob"
      return names.find_index(element)
    end
  end
  return -1
end


def dis(price, discount)
	price = discount.to_f/100
	return price.round(2)
end


#p find_bob(["Jimmy", "Layla", "Bob"])
#p find_bob(["Bob", "Layla", "Kaitlyn", "Patricia"])
#p find_bob(["Jimmy", "Layla", "James"])

p dis(100,75)
