require 'fileutils'

REGEX = /namespace[\W]+(\w+)/

def processcontent(content, outfile, parent)
  curlyopen = content.index("{")
  curlyclose = content.index("}")
  if curlyopen && curlyclose
    prefix = content[0..(curlyopen - 1)]
    postfix = content[(curlyclose + 1)..]
    capture = content[curlyopen..curlyclose]
  end
end

def processcontent(content, outfile, parent)
  first = content.index("{")
  lasti = content.rindex("}")
  if first && lasti
    prefix = content[0..(first - 1)]
    postfix = content[(lasti + 1)..]
    capture = content[first..lasti]
    nsmatch = prefix.match(REGEX)
    if capture.length > 2
      inside = capture[1..-2]
      if nsmatch
        nsname = nsmatch[1]
        outfile.puts "%s -- %s;" % [nsname, parent]
        processcontent(inside, outfile, nsname)
      end
    end
    if postfix.length > 0
      processcontent(postfix, outfile, parent)
    end
  end
end

def processfile(dpath, filename)
  basename = File.basename(filename, ".h")
  filenameout = basename + ".dot"
  filepathout = File.join(dpath, filenameout)

  puts "Process file for %s" % basename

  File.delete(filepathout) if File.exist?(filepathout)

  File.open(filepathout, "w") { |fout |
    fout.puts "graph %s {" % basename
    File.open(filename, "r") { | fin |
      nsmap = Hash.new
      processcontent(fin.read, fout, "root")
    }
    fout.puts "}"
  }
end

# Determine and create the destination path
destpath = File.join(__dir__, "graphviz")
FileUtils.mkdir_p destpath

# Determine and list the input files
srcpath = File.join(__dir__, "src", "*.h")
srcdir = Dir[srcpath]

# Process each file names or paths from the input list
srcdir.each { |item| 
  processfile(destpath, item)
}
