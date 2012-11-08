from xml.dom.minidom import parseString

print " =======================\n TELEMETRY ANALYSIS \n ======================="

typeSizes = { 	"float" 	: 32,
		"uint16" 	: 16,
		"int16" 	: 16,
		"uint32"	: 32,
		"int32"		: 32,
		"uint8" 	: 8,
		"int8" 		: 8,
		"int8[]" 	: 8*8,
		"uint8[]" 	: 8*8,
		"uint16[]" 	: 8*16,
		"int16[]" 	: 8*16,
		"uint32[]" 	: 8*32,
		"double" 	: 64,
		"string"	: 255,}

class Message:
	def __init__(self, name, id):
		self.name = name
		self.id = id
		self.fields = {}
		self.size = 0

class Field:
	def __init__(self, name, type):
		self.name = name
		self.type = type

class TelemetryMode:
	def __init__(self, name):
		self.name = name
		self.entries = {}

class TelemetryEntry:
	def __init__(self, message, period):
		self.message = message
		self.period = period

class TelemetryParser:
	def __init__(self, telemetryFile, messageFile):
		self.messages = {}
		self.telemetryModes = {}

		msgFile = open(messageFile,'r')
		msgData = msgFile.read()
		msgFile.close()

		tmFile = open(telemetryFile,'r')
		tmData = tmFile.read()
		tmFile.close()

		msgDom = parseString(msgData)
		msgs = msgDom.getElementsByTagName('message')
		for msg in msgs:
			if msg.getAttribute('name') != " ":
				m = Message(msg.getAttribute('name'),msg.getAttribute('id'))
				mSize = 0
				fields = msg.getElementsByTagName('field')
				for field in fields:
					f = Field(field.getAttribute('name'),field.getAttribute('type'))
					m.fields[field.getAttribute('name')] = f
					mSize += typeSizes[f.type]
				m.size = mSize
				self.messages[msg.getAttribute('name')] = m
				print m.name + "  \t\tSize: " + str(m.size) + " bits"

		tmDom = parseString(tmData)
		prcs = tmDom.getElementsByTagName('process')
		modes  = prcs[0].getElementsByTagName('mode')
		for mode in modes:
			tmMode = TelemetryMode(mode.getAttribute('name'))
			entries  = mode.getElementsByTagName('message')
			for entry in entries:
				tmEntry = TelemetryEntry(self.messages[entry.getAttribute('name')],entry.getAttribute('period'))
				tmMode.entries[entry.getAttribute('name')] = tmEntry
#				print tmEntry.message.name + "  \t\tSize: " + str(tmEntry.message.size) + " bits \t Period: " + str(tmEntry.period) + " seconds  (F = " +  str(int(1)/float(tmEntry.period)) + " Hz)"
				#print tmEntry.message.name + "  \t" + str(tmEntry.message.size) + "\t" + str(tmEntry.period) + "\t" +  str(int(1)/float(tmEntry.period)) + ""
			self.telemetryModes[mode.getAttribute('name')] = tmMode


class TelemetryAnalyzer:
	def __init__(self, tmpa, modeName):
		self.tmpa = tmpa
		self.modeName = modeName
		self.telemetryEntries = tmpa.telemetryModes[modeName].entries

#	def calculateMessageSizes(self):
		#for mName,mObj in tmpa.messages:
			


tmpa = TelemetryParser("/home/dirk/nonSynced/paparazzi/conf/telemetry/default_fixedwing.xml","/home/dirk/nonSynced/paparazzi/conf/messages.xml")
tma = TelemetryAnalyzer(tmpa,'default')

