// Copyright (c) 2019 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

#ifndef SRC_NR_MODEL_NR_CONTROL_MESSAGES_H_
#define SRC_NR_MODEL_NR_CONTROL_MESSAGES_H_

#include "nr-phy-mac-common.h"
#include "nr-rrc-sap.h"

#include <ns3/simple-ref-count.h>

namespace ns3
{

/**
 * \ingroup utils
 * \brief Available TDD slot types. Ordering is important.
 */
enum LteNrTddSlotType : uint8_t
{
    DL = 0, //!< DL CTRL + DL DATA
    S = 1,  //!< DL CTRL + DL DATA + UL CTRL
    F = 2,  //!< DL CTRL + DL DATA + UL DATA + UL CTRL
    UL = 3, //!< UL DATA + UL CTRL
};

std::ostream& operator<<(std::ostream& os, const LteNrTddSlotType& item);

/**
 * \ingroup utils
 * \brief The NrControlMessage class
 *
 * Base class for all the messages types that the UE and the GNB can exchange. The
 * use is not usually involved in the message creation; however, you can read them
 * with the trace sources that are at your disposal in the MAC and PHY classes.
 */
class NrControlMessage : public SimpleRefCount<NrControlMessage>
{
  public:
    /**
     * \brief The Message Type
     */
    enum messageType
    {
        UL_DCI,        //!< The resources allocation map from the BS to the attached UEs (UL)
        DL_DCI,        //!< The resources allocation map from the BS to the attached UEs (DL)
        DL_CQI,        //!< DL CQI message
        MIB,           //!< Master Information Block
        SIB1,          //!< System Information Block Type 1
        RACH_PREAMBLE, //!< Random Access Preamble
        RAR,           //!< Random Access Response
        BSR,           //!< Buffer Status Report
        DL_HARQ,       //!< DL HARQ feedback
        SR,            //!< Scheduling Request: asking for space
        SRS,           //!< SRS
    };

    /**
     * \brief NrControlMessage
     */
    NrControlMessage();
    /**
     * \brief ~NrControlMessage
     */
    virtual ~NrControlMessage();

    /**
     * \brief Get the MessageType
     * \return the message type
     */
    messageType GetMessageType() const;

    /**
     * \brief Set the BWP in which this message has been generated
     * \param bwpId the BwpId
     */
    void SetSourceBwp(uint16_t bwpId);

    /**
     * \return the BWP in which this message has been generated
     *
     * If SetSourceBwp() is not called beforehand, the method will ABORT.
     */
    uint16_t GetSourceBwp() const;

  protected:
    /**
     * \brief Set the MessageType
     * \param type type of the message
     */
    void SetMessageType(messageType type);

  private:
    messageType m_messageType; //!< The message type
    int32_t m_bwpId{-1};       //!< Source BWP.
};

/**
 * \ingroup utils
 * \brief SR message
 *
 * Message that represent a scheduling request, with the RNTI from
 * which this message is coming.
 * 
 * NrSrMessage에 큐 같이 보내기
 */
class NrSRMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrSRMessage constructor
     */
    NrSRMessage();
    /**
     * \brief ~NrSRMessage
     */
    ~NrSRMessage() override;

    /**
     * \brief Set the RNTI to which this message is intended
     * \param rnti RNTI
     */
    void SetRNTI(uint16_t rnti);

    /**
     * \brief Get the RNTI of this message
     * \return RNTI
     */

    uint16_t GetRNTI() const;
    void SetPacketCreationTimes(const std::queue<uint64_t>& times)
    {
      m_packet_creation_time_queue=times;
    }
    std::queue<uint64_t> GetPacketCreationTimes() const
    {
      return m_packet_creation_time_queue;  
    }
  private:
    uint16_t m_rnti{0}; //!< RNTI
    std::queue<uint64_t> m_packet_creation_time_queue;  // 패킷 생성 시간 큐
};

/**
 * \brief The message that represents a DL DCI message
 * \ingroup utils
 */
class NrDlDciMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrDlDciMessage constructor
     * \param dci the DCI
     */
    NrDlDciMessage(const std::shared_ptr<DciInfoElementTdma>& dci);
    /**
     * \brief ~NrDlDciMessage
     */
    ~NrDlDciMessage() override;

    /**
     * \brief Get the DCI
     * \return the DCI
     */
    std::shared_ptr<DciInfoElementTdma> GetDciInfoElement();

    /**
     * \brief Set the delay (in slots) between DL DCI
     * reception and subframe to which it applies for
     * reception/transmission of Data (k0)
     */
    void SetKDelay(uint32_t delay);
    /**
     * \brief Get the delay (in slots) between DCI
     * reception and subframe to which it applies for
     * reception/transmission of Data (k0)
     * \return k delay
     */
    uint32_t GetKDelay() const;

    /**
     * \brief Set the delay (in slots) between DL Data
     * reception and subframe to which it applies for
     * Harq feedback
     *
     * Note that K1 delay is also passed with the UL DCI
     * however the UE ignores it (applies only for DL DCI)
     */
    void SetK1Delay(uint32_t delay);
    /**
     * \brief Get the delay (in slots) between DL Data
     * reception and subframe to which it applies for
     * Harq feedback
     * \return k1 delay
     */
    uint32_t GetK1Delay() const;

  private:
    uint32_t m_k; //!< delay (in slots) between DL/UL DCI reception and subframe to which it applies
                  //!< for reception/transmission of Data (k0/k2)
    uint32_t m_k1; //!< delay (in slots) between DL Data reception and subframe to which it applies
                   //!< for Harq feedback (k1)
    std::shared_ptr<DciInfoElementTdma> m_dciInfoElement;
};

/**
 * \brief The message that represents a UL DCI message
 * \ingroup utils
 */
class NrUlDciMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrUlDciMessage constructor
     * \param dci DCI
     */
    NrUlDciMessage(const std::shared_ptr<DciInfoElementTdma>& dci);
    /**
     * \brief ~NrUlDciMessage
     */
    ~NrUlDciMessage() override;

    /**
     * \brief Get the DCI
     * \return the DCI
     */
    std::shared_ptr<DciInfoElementTdma> GetDciInfoElement();

    /**
     * \brief Set the delay (in slots) between UCI
     * reception and subframe to which it applies for
     * reception/transmission of Data (k2)
     */
    void SetKDelay(uint32_t delay);
    /**
     * \brief Get the delay (in slots) between UCI
     * reception and subframe to which it applies for
     * reception/transmission of Data (k2)
     * \return k delay
     */
    uint32_t GetKDelay() const;

  private:
    uint32_t m_k; //!< delay (in slots) between UCI reception and subframe to which it applies for
                  //!< reception/transmission of Data (k2)
    std::shared_ptr<DciInfoElementTdma> m_dciInfoElement; //!< the DCI
};

/**
 * \brief The message that represents a DL CQI message
 * \ingroup utils
 */
class NrDlCqiMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrDlCqiMessage constructor
     */
    NrDlCqiMessage();
    /**
     * \brief ~NrDlCqiMessage
     */
    ~NrDlCqiMessage() override;

    /**
     * \brief Set the DlCqi to transmit
     * \param cqi the DlCqi info
     */
    void SetDlCqi(DlCqiInfo cqi);
    /**
     * \brief Get the DlCqi in this message
     * \return the DL CQI
     */
    DlCqiInfo GetDlCqi();

  private:
    DlCqiInfo m_cqi; //!< The DlCqiInfo struct
};

/**
 * \ingroup utils
 * \brief the BSR message
 *
 * The uplink BsrNrControlMessage defines the specific
 * extension of the CE element for reporting the buffer status report
 * 
 * NrBsrMessage에 패킷 생성시간 queue 및 관련 메서드(set,get) 구성
 */
class NrBsrMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrBsrMessage constructor
     */
    NrBsrMessage();
    /**
     * \brief ~NrBsrMessage
     */
    ~NrBsrMessage() override;

    /**
     * \brief add a BSR feedback record into the message.
     * \param bsr the BSR feedback
     */
    void SetBsr(MacCeElement bsr);

    /**
     * \brief Get BSR information
     * \return BSR message
     */
    MacCeElement GetBsr();

    /**
     * PacketCreationTimeQueue Set,Get Method
     */
    void SetRnti(uint16_t rnti)
    {
      bsr_rnti = rnti;
    }
    uint16_t GetRnti()
    {
      return bsr_rnti;
    }
  private:
    MacCeElement m_bsr; //!< The BSR
    uint16_t bsr_rnti;  // BsrMessage rnti
};

// ---------------------------------------------------------------------------

/**
 * \ingroup utils
 * \brief Abstract model for broadcasting the Master Information Block (MIB)
 *        within the control channel (BCCH).
 *
 */
class NrMibMessage : public NrControlMessage
{
  public:
    /**
     * \brief Create a new instance of MIB control message.
     */
    NrMibMessage();

    /**
     * \brief Replace the MIB content of this control message.
     * \param mib the desired MIB content
     */
    void SetMib(NrRrcSap::MasterInformationBlock mib);

    /**
     * \brief Retrieve the MIB content from this control message.
     * \return the current MIB content that this control message holds
     */
    NrRrcSap::MasterInformationBlock GetMib() const;

  private:
    NrRrcSap::MasterInformationBlock m_mib; //!< The MIB
};

// ---------------------------------------------------------------------------

/**
 * \ingroup utils
 * \brief Abstract model for broadcasting the System Information Block Type 1
 *        (SIB1) within the control channel (BCCH).
 *
 */
class NrSib1Message : public NrControlMessage
{
  public:
    /**
     * \brief Create a new instance of SIB1 control message.
     */
    NrSib1Message();

    /**
     * \brief Replace the SIB1 content of this control message.
     * \param sib1 the desired SIB1 content
     */
    void SetSib1(NrRrcSap::SystemInformationBlockType1 sib1);

    /**
     * \brief Retrieve the SIB1 content from this control message.
     * \return the current SIB1 content that this control message holds
     */
    NrRrcSap::SystemInformationBlockType1 GetSib1() const;

  private:
    NrRrcSap::SystemInformationBlockType1 m_sib1; //!< Sib1 content
};

// ---------------------------------------------------------------------------

/**
 * \ingroup utils
 *
 * \brief Abstract model for the Random Access Preamble
 */
class NrRachPreambleMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrRachPreambleMessage constructor
     */
    NrRachPreambleMessage();

    /**
     * \brief ~NrRachPreambleMessage
     */
    ~NrRachPreambleMessage() override;

    /**
     * Set the Random Access Preamble Identifier (RAPID), see 3GPP TS 36.321 6.2.2
     *
     * \param rapid the RAPID
     */
    void SetRapId(uint32_t rapid);

    /**
     *
     * \return the RAPID
     */
    uint32_t GetRapId() const;

  private:
    uint32_t m_rapId; //!< The RAP ID
};

// ---------------------------------------------------------------------------

/**
 * \ingroup utils
 *
 * \brief Abstract model for the MAC Random Access Response message
 */
class NrRarMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrRarMessage constructor
     */
    NrRarMessage();

    /**
     * \brief ~NrRarMessage
     */
    ~NrRarMessage() override;

    /**
     *
     * \param raRnti the RA-RNTI, see 3GPP TS 36.321 5.1.4
     */
    void SetRaRnti(uint16_t raRnti);

    /**
     *
     * \return  the RA-RNTI, see 3GPP TS 36.321 5.1.4
     */
    uint16_t GetRaRnti() const;

    /**
     * a MAC RAR and the corresponding RAPID subheader
     *
     */
    struct Rar
    {
        uint8_t rapId;                      //!< RA ID
        NrBuildRarListElement_s rarPayload; //!< RA Payload
    };

    /**
     * add a RAR to the MAC PDU, see 3GPP TS 36.321 6.2.3
     *
     * \param rar the rar
     */
    void AddRar(Rar rar);

    /**
     *
     * \return a const iterator to the beginning of the RAR list
     */
    std::list<Rar>::const_iterator RarListBegin() const;

    /**
     *
     * \return a const iterator to the end of the RAR list
     */
    std::list<Rar>::const_iterator RarListEnd() const;

  private:
    std::list<Rar> m_rarList; //!< RAR List
    uint16_t m_raRnti;        //!< RNTI
};

/**
 * \ingroup utils
 * \brief DlHarqFeedback message
 *
 * The downlink NrDlHarqFeedbackMessage defines the specific
 * messages for transmitting the DL HARQ feedback through PUCCH
 */
class NrDlHarqFeedbackMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrDlHarqFeedbackMessage constructor
     */
    NrDlHarqFeedbackMessage();
    /**
     * \brief ~NrDlHarqFeedbackMessage
     */
    ~NrDlHarqFeedbackMessage() override;

    /**
     * \brief add a DL HARQ feedback record into the message.
     * \param m the DL HARQ feedback
     */
    void SetDlHarqFeedback(DlHarqInfo m);

    /**
     * \brief Get DL HARQ information
     * \return DL HARQ message
     */
    DlHarqInfo GetDlHarqFeedback();

  private:
    DlHarqInfo m_dlHarqInfo; //!< DL Harq Info
};

/**
 * \ingroup utils
 * \brief NrSrsMessage message
 *
 */
class NrSrsMessage : public NrControlMessage
{
  public:
    /**
     * \brief NrDlHarqFeedbackMessage constructor
     */
    NrSrsMessage();
    /**
     * \brief ~NrDlHarqFeedbackMessage
     */
    ~NrSrsMessage() override = default;
};

} // namespace ns3

#endif /* SRC_NR_MODEL_NR_CONTROL_MESSAGES_H_ */
