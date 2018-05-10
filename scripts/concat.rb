def read_file(filepath)
    line_num=0
    text=File.open(filepath).read
    text.gsub!(/\r\n?/, "\n")
    lines = []
    text.each_line do |line|
        lines << line
    end
    puts "Total: #{lines.size}"
    lines.shuffle
end

def write_file(filepath, lines)
    file = File.new(filepath, "w")
    lines.each do |line|
        file.puts line
    end
    file.close
end

target_count = 3000
root_folder = "/home/gustavoneves/data/gemini/"
source_folder = "train/01_source/"
targets = ["ssiv_bahia", "jequitaia", "balsa"]
filename = "test.txt"

lines = []
targets.each do |t|
    filepath = File.join(root_folder, t, source_folder, filename)
    puts "Read file: #{filepath}"
    if File.exists?(filepath)
        l = read_file(filepath)
        if l.size-1 > target_count
            lines = lines+l.first(target_count)
        else
            lines = lines+l
        end
    end
end
filepath = File.join(root_folder, filename)
write_file(filepath, lines.shuffle)
puts "Concat file with success!"
