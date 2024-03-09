import os
import smtplib
import mimetypes
from email.MIMEMultipart import MIMEMultipart
from email.MIMEBase import MIMEBase
from email.MIMEText import MIMEText
from email.MIMEAudio import MIMEAudio
from email.MIMEImage import MIMEImage
from email.Encoders import encode_base64

def sendMail(subject, text, user, recipient, password, smtp_server, attachmentFilePath):
    gmailUser = user
    gmailPassword = password

    msg = MIMEMultipart()
    msg['From'] = 'sbdservice@sbd.iridium.com'
    msg['To'] = recipient
    msg['Subject'] = subject
    msg.attach(MIMEText(text))


    msg.attach(getAttachment(attachmentFilePath))

    mailServer = smtplib.SMTP(smtp_server, 587)
    mailServer.ehlo()
    mailServer.starttls()
    mailServer.ehlo()
    mailServer.login(gmailUser, gmailPassword)
    mailServer.sendmail(gmailUser, recipient, msg.as_string())
    mailServer.close()

    print('Sent email to %s' % recipient)

def getAttachment(attachmentFilePath):
    contentType, encoding = mimetypes.guess_type(attachmentFilePath)

    if contentType is None or encoding is not None:
        contentType = 'application/octet-stream'

    mainType, subType = contentType.split('/', 1)
    file = open(attachmentFilePath, 'rb')

    if mainType == 'text':
        attachment = MIMEText(file.read())
    elif mainType == 'message':
        attachment = email.message_from_file(file)
    elif mainType == 'image':
        attachment = MIMEImage(file.read(),_subType=subType)
    elif mainType == 'audio':
        attachment = MIMEAudio(file.read(),_subType=subType)
    else:
        attachment = MIMEBase(mainType, subType)
        attachment.set_payload(file.read())
        encode_base64(attachment)

    file.close()

    attachment.add_header('Content-Disposition', 'attachment',   filename=os.path.basename(attachmentFilePath))
    return attachment
    

