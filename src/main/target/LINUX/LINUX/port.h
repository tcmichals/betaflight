class Port
{


public:
    serialPort_t port;
    std::vector<uint8_t> rxBuffer;
    std::vector<uint8_t> txBuffer;
    std:: mutex         txLock;
    std::mutex           rxLock;

    bool connected;
    uint16_t clientCount;
    uint8_t id;
    
    serialPortVTable    m_callbackVTable;
    serialPort_t        &m_entry;
public:
Port(serialPort_t &serialPortEntry);
~Port();


};

inline 
Port::Port(serialPort_t &entry): m_entry(entry)
{
    memset(&m_callbackVTable, 0, sizeof(m_callbackVTable));
    memset(&m_entry, 0, sizeof(m_entry));
    m_entry.vTable = &m_callbackVTable;
}

inline 
Port::~Port()
{

}